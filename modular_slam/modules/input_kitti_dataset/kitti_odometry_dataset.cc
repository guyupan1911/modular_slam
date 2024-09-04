/**
 * @file kitti_odometry_dataset.cc
 * @author guyupan
 * @brief kitti odometrty dataset
 * @version 0.1
 * @date 2024-09-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "modules/input_kitti_dataset/kitti_odometry_dataset.h"

#include "mrpt/system/filesystem.h" //ASSERT_DIRECTORY_EXISTS_()
#include "mrpt/math/CVectorDynamic.h"
#include "mrpt/system/CDirectoryExplorer.h"
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>

#include "Eigen/Dense"

#include "modules/yaml/yaml_helper.h"

namespace modular_slam {

// make a list of sorted files
static void BuildListFiles(const std::string& dir,
  const std::string& file_extension, std::vector<std::string>& out_lst) {
  out_lst.clear();
  if (!mrpt::system::directoryExists(dir)) {
    return;
  }

  using direxpl = mrpt::system::CDirectoryExplorer;
  direxpl::TFileInfoList lstFiles;
  direxpl::explore(dir, FILE_ATTRIB_ARCHIVE, lstFiles);
  direxpl::sortByName(lstFiles);
  direxpl::filterByExtension(lstFiles, file_extension);
  out_lst.resize(lstFiles.size());
  std::transform(
    lstFiles.begin(), lstFiles.end(), out_lst.begin(),
    [](auto& fil) { return fil.name; });
}

static void ParseCalibLine(const std::string& line, Eigen::Matrix<double, 3, 4>& M) {
  std::istringstream ss(line);
  for (Eigen::Index r = 0; r < M.rows(); r++) {
    for (Eigen::Index c = 0; c < M.cols(); c++) {
      if (!(ss >> M(r, c))) {
          THROW_EXCEPTION_FMT("Error parsing calib line: `%s`", line.c_str());
      }
    }
  }
}

KittiOdometryDataset::KittiOdometryDataset() {
  this->mrpt::system::COutputLogger::setLoggerName("KittiDoometryDataset");
}

void KittiOdometryDataset::Initialize_rds(const mrpt::containers::yaml& yaml) {
  MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << yaml);

  // mandatory parameters
  ENSURE_YAML_ENTRY_EXISTS(yaml, "params");
  auto cfg = yaml["params"];

  YAML_LOAD_MEMBER_REQ(base_dir, std::string);
  YAML_LOAD_MEMBER_REQ(sequence, std::string);
  YAML_LOAD_MEMBER_OPT(clouds_as_organized_points, bool);

  seq_dir_ = base_dir_ + "/sequences/" + sequence_;
  ASSERT_DIRECTORY_EXISTS_(seq_dir_);

  // optional params with default values
  publish_lidar_ = cfg.getOrDefault<bool>("publish_lidar", publish_lidar_);
  
  publish_ground_truth_ = cfg.getOrDefault<bool>("publish_ground_truth", publish_ground_truth_);

  for (unsigned int i = 0; i < 4; i++) {
    publish_image_[i] = cfg.getOrDefault<bool>(
      mrpt::format("publish_image_%u", i), publish_image_[i]);
  }
  
  MRPT_LOG_INFO_STREAM("Loading Kitti dataset from: " << seq_dir_);
  // load timestamps:
  {
    mrpt::math::CVectorDynamic<double> mtimes;
    mtimes.loadFromTextFile(seq_dir_ + std::string("/times.txt"));
    lst_timestamps_.resize(static_cast<std::size_t>(mtimes.size()));
    Eigen::VectorXd::Map(&lst_timestamps_[0], mtimes.size()) = mtimes.asEigen();
  }
  const auto N = lst_timestamps_.size();
  MRPT_LOG_DEBUG_STREAM("Dataset timespecs: " << N);

  // load sensor data
  BuildListFiles(seq_dir_ + "/velodyne", "bin", lst_velodyne_);
  if (!lst_velodyne_.empty()) {
    ASSERTMSG_(lst_velodyne_.size() == N, "Velodyne: invalid file count");
  }

  MRPT_LOG_INFO_STREAM(
    "Velodyne pointclouds: "
    << (!lst_velodyne_.empty()
      ? "Found (" + std::to_string(lst_velodyne_.size()) + ")"
      : "Not found"));

  for (unsigned int i = 0; i < 4; i++) {
    BuildListFiles(seq_dir_ + "/image_" + std::to_string(i), "png", lst_image_[i]);
    if (!lst_image_[i].empty()) {
      ASSERTMSG_(
        lst_image_[i].size() == N,
        mrpt::format("image_%u: invalid file count", i));
    }

    MRPT_LOG_INFO_STREAM(
      "Camera channel `image_"
      << i << "`: "
      << (!lst_image_[i].empty()
          ? "Found (" + std::to_string(lst_image_[i].size()) + ")"
          : "Not found"));

    // Override user choice if not possible to publish images:
    if (lst_image_[i].empty()){
      publish_image_[i] = false;
    }
  }

  // load sensor calibration
  const std::string fil_calib = seq_dir_ + std::string("/calib.txt");
  ASSERT_FILE_EXISTS_(fil_calib);

  // load projection matrices:
  auto calib = mrpt::containers::yaml::FromFile(fil_calib);
  ENSURE_YAML_ENTRY_EXISTS(calib, "P0");
  ENSURE_YAML_ENTRY_EXISTS(calib, "P1");
  ENSURE_YAML_ENTRY_EXISTS(calib, "P2");
  ENSURE_YAML_ENTRY_EXISTS(calib, "P3");
  ENSURE_YAML_ENTRY_EXISTS(calib, "Tr");

  Eigen::Matrix<double, 3, 4> P[4], Tr;
  ParseCalibLine(calib["Tr"].as<std::string>(), Tr);
  for (unsigned int i = 0; i < 4; i++) {
    ParseCalibLine(calib["P" + std::to_string(i)].as<std::string>(), P[i]);
  }

  // load camera intrinsics:
  for (unsigned int i = 0; i < 4; i++) {
    const double fx = P[i](0, 0), fy = P[i](1, 1), cx = P[i](0, 2),
                  cy = P[i](1, 2);
    cam_intrinsics_[i].setIntrinsicParamsFromValues(fx, fy, cx, cy);

    // load resolution: try loading the first image:
    if (!lst_image_[i].empty()) {
      mrpt::img::CImage im;
      if (im.loadFromFile(
        seq_dir_ + std::string("/image_") + std::to_string(i) +
        std::string("/") + lst_image_[i][0])) {
        cam_intrinsics_[i].ncols = static_cast<uint32_t>(im.getWidth());
        cam_intrinsics_[i].nrows = static_cast<uint32_t>(im.getHeight());
        MRPT_LOG_DEBUG_STREAM(
            "image_"
            << i << ": detected image size=" << cam_intrinsics_[i].ncols
            << "x" << cam_intrinsics_[i].nrows);
      }
    }

    // camera extrinsic params/ pose wrt vehicle origin:
    cam_poses_[i]   = mrpt::math::TPose3D::Identity();
    cam_poses_[i].x = -P[i](0, 3) / P[i](0, 0);
  }

  // velodyne is the (0,0,0) of the vehicle.
  // image_0 pose wrt velo is "Tr":
  auto Trh = mrpt::math::CMatrixDouble44::Identity();
  Trh.block<3, 4>(0, 0) = Tr;
  MRPT_LOG_DEBUG_STREAM("Original Trh= (velo wrt cam_0) \n" << Trh);
  // Inverse:
  Trh = Trh.inverse();
  MRPT_LOG_DEBUG_STREAM("Inverted Trh= (cam_0 wrt velo) \n" << Trh);

  // Camera 0:
  cam_poses_[0] = mrpt::poses::CPose3D(Trh).asTPose();

  // Cameras 1-3:
  for (unsigned int i = 1; i < 4; i++) {
    cam_poses_[0].composePose(cam_poses_[i], cam_poses_[i]);
  }

  // Debug: dump poses.
  for (unsigned int i = 0; i < 4; i++) {
    mrpt::poses::CPose3D        p(cam_poses_[i]);
    mrpt::math::CMatrixDouble44 T;
    p.getHomogeneousMatrix(T);
    MRPT_LOG_DEBUG_STREAM(
        "image_" << i << " pose on vehicle: " << cam_poses_[i]
                  << "\nTransf. matrix:\n"
                  << T);
  }
  
  // Load ground truth poses, if available:
  const auto gtFile = base_dir_ + "/poses/" + sequence_ + ".txt";
  if (mrpt::system::fileExists(gtFile)) {
    ground_truth_poses_.loadFromTextFile(gtFile);

    ASSERT_EQUAL_(ground_truth_poses_.cols(), 12U);
    ASSERT_EQUAL_(static_cast<size_t>(ground_truth_poses_.rows()), lst_timestamps_.size());

    // Convert into the format expected by MOLA generic interface:
    mrpt::math::CMatrixDouble44 m = mrpt::math::CMatrixDouble44::Identity();
    const auto cam0PoseInv        = -mrpt::poses::CPose3D(cam_poses_.at(0));

    using namespace mrpt::literals;  // _deg
    const auto axisChange = mrpt::poses::CPose3D::FromYawPitchRoll(
        -90.0_deg, 0.0_deg, -90.0_deg);

    for (size_t i = 0; i < lst_timestamps_.size(); i++) {
      for (int row = 0, ij_idx = 0; row < 3; row++) {
        for (int col = 0; col < 4; col++, ij_idx++) {
          m(row, col) = ground_truth_poses_(i, ij_idx);
        }
      }

      // ground truth is for cam0:
      const auto gtCam0Pose = mrpt::poses::CPose3D::FromHomogeneousMatrix(m);

      // Convert it to the vehicle frame, for consistency with all MOLA
      // datasets:
      const auto gtPose = axisChange + gtCam0Pose + cam0PoseInv;

      ground_truth_trajectory_.insert(mrpt::Clock::fromDouble(lst_timestamps_.at(i)), gtPose);
    }

    MRPT_LOG_INFO("Ground truth poses: Found");
  } else {
    MRPT_LOG_WARN_STREAM("Ground truth poses: not found. Expected file: " << gtFile);
  }

  initialized_ = true;
}

size_t KittiOdometryDataset::DatasetSize() const {
  ASSERT_(initialized_);
  return lst_timestamps_.size();
}

mrpt::obs::CSensoryFrame::Ptr KittiOdometryDataset::GetObservations(size_t timestamp) const {
  
  auto sensor_frame = mrpt::obs::CSensoryFrame::Create();

  for (size_t i = 0; i < publish_image_.size(); ++i) {
    if (!publish_image_[i]) {
      continue;
    }
    sensor_frame->insert(GetImage(i, timestamp));
  }

  if (publish_lidar_) {
    sensor_frame->insert(GetPointCloud(timestamp));
  }

  return sensor_frame;
}

std::shared_ptr<mrpt::obs::CObservationImage> KittiOdometryDataset::GetImage(
  const unsigned int cam_idx, size_t step) const {
  
  ASSERT_(initialized_);
  ASSERT_LT_(step, lst_timestamps_.size());

  LoadImage(cam_idx, step);
  auto o = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(
      read_ahead_image_obs_.at(step).at(cam_idx));
  ASSERT_(o);
  return o;
}

void KittiOdometryDataset::LoadImage(const unsigned int cam_idx, const size_t step) const {
  
  // unload old observations
  AutoUnloadOldEntries();

  if (read_ahead_image_obs_[step][cam_idx]) {
    return;
  }

  auto obs = mrpt::obs::CObservationImage::Create();
  obs->sensorLabel = std::string("image_") + std::to_string(cam_idx);

  const auto f = seq_dir_ + std::string("/image_") + std::to_string(cam_idx) +
                 std::string("/") + lst_image_[cam_idx][step];
  obs->image.setExternalStorage(f);

  obs->image.forceLoad();

  obs->cameraParams = cam_intrinsics_[cam_idx];
  obs->setSensorPose(mrpt::poses::CPose3D(cam_poses_[cam_idx]));
  obs->timestamp = mrpt::Clock::fromDouble(lst_timestamps_.at(step));

  auto o = mrpt::ptr_cast<mrpt::obs::CObservation>::from(obs);
  read_ahead_image_obs_[step][cam_idx] = std::move(o);
}

std::shared_ptr<mrpt::obs::CObservation> KittiOdometryDataset::GetPointCloud(
  size_t step) const {

  ASSERT_(initialized_);
  ASSERT_LT_(step, lst_timestamps_.size());

  LoadLidar(step);
  auto o = read_ahead_lidar_obs_.at(step);
  return o;
}

void KittiOdometryDataset::LoadLidar(size_t step) const {
  
  AutoUnloadOldEntries();

  if (read_ahead_lidar_obs_[step]) {
    return;
  }

  // load velodyne pointcloud:
  const auto f = seq_dir_ + std::string("/velodyne/") + lst_velodyne_[step];

  auto obs = mrpt::obs::CObservationPointCloud::Create();
  obs->sensorLabel = "lidar";
  obs->setAsExternalStorage(
      f,
      mrpt::obs::CObservationPointCloud::ExternalStorageFormat::KittiBinFile);
  obs->load();  // force loading now from disk
  ASSERTMSG_(
      obs->pointcloud,
      mrpt::format("Error loading kitti scan file: '%s'", f.c_str()));
  
  // Correct wrong intrinsic calibration in the original kitti datasets:
  // Refer to these works & implementations (on which this solution is based
  // on):
  // - IMLS-SLAM
  // - CT-ICP
  // - KISS-ICP
  //
  // See:
  // "IMLS-SLAM: scan-to-model matching based on 3D data", JE Deschaud, 2018.
  //

  // We need to "elevate" each point by this angle: VERTICAL_ANGLE_OFFSET
  if (VERTICAL_ANGLE_OFFSET != 0) {
    // Due to the ring-like, rotating nature of 3D LIDARs, we cannot do this
    // in any more efficient way than go through the points one by one:
    auto& xs = obs->pointcloud->getPointsBufferRef_x();
    auto& ys = obs->pointcloud->getPointsBufferRef_y();
    auto& zs = obs->pointcloud->getPointsBufferRef_z();

    const Eigen::Vector3d uz(0., 0., 1.);
    for (size_t i = 0; i < xs.size(); i++) {
      const Eigen::Vector3d pt(xs[i], ys[i], zs[i]);
      const Eigen::Vector3d rotationVector = pt.cross(uz);

      const auto aa = Eigen::AngleAxisd(
          VERTICAL_ANGLE_OFFSET, rotationVector.normalized());
      const Eigen::Vector3d newPt = aa * pt;

      obs->pointcloud->setPoint(i, {newPt.x(), newPt.y(), newPt.z()});
    }
  }

  // Pose: velodyne is at the origin of the vehicle coordinates in
  // Kitti datasets.
  obs->sensorPose = mrpt::poses::CPose3D();
  obs->timestamp  = mrpt::Clock::fromDouble(lst_timestamps_.at(step));

  mrpt::obs::CObservation::Ptr o;
  // Now, publish it as pointcloud or as an organized cloud:
  if (!clouds_as_organized_points_) {
    // we are done:
    o = std::dynamic_pointer_cast<mrpt::obs::CObservation>(obs);
  } else {
    auto rs         = mrpt::obs::CObservationRotatingScan::Create();
    rs->sensorPose  = obs->sensorPose;
    rs->sensorLabel = obs->sensorLabel;
    rs->timestamp   = obs->timestamp;

    rs->sweepDuration = .0;  // [sec] for already de-skewed scans
    rs->lidarModel    = "HDL-64E";
    rs->minRange      = 0.1;
    rs->maxRange      = 120.0;

    rs->columnCount     = range_matrix_column_count_;
    rs->rowCount        = range_matrix_row_count_;
    rs->rangeResolution = 5e-3;  // 5 mm

    rs->organizedPoints.resize(rs->rowCount, rs->columnCount);
    rs->intensityImage.resize(rs->rowCount, rs->columnCount);
    rs->rangeImage.resize(rs->rowCount, rs->columnCount);

    auto ptsXYZI = std::dynamic_pointer_cast<mrpt::maps::CPointsMapXYZI>(
        obs->pointcloud);
    ASSERT_(ptsXYZI);

    const auto& xs = ptsXYZI->getPointsBufferRef_x();
    const auto& ys = ptsXYZI->getPointsBufferRef_y();
    const auto& zs = ptsXYZI->getPointsBufferRef_z();

    const size_t nPts = xs.size();

    // Based on:
    // https://github.com/TixiaoShan/LIO-SAM/blob/master/config/doc/kitti2bag/kitti2bag.py

    // (JLBC) Note that this code assumes scan deskew has not been already
    // applied!

    const float fov_down = mrpt::DEG2RAD(-24.8f);
    const float fov      = mrpt::DEG2RAD(std::abs(-24.8f) + abs(2.0f));

    for (size_t i = 0; i < nPts; i++) {
      // intensity comes normalized [0,1]
      const float ptInt = ptsXYZI->getPointIntensity(i);

      const float range_xy =
          std::sqrt(mrpt::square(xs[i]) + mrpt::square(ys[i]));
      const float pitch = std::asin(zs[i] / range_xy);
      const float yaw   = std::atan2(ys[i], xs[i]);

      float     proj_y = (pitch + abs(fov_down)) / fov;  // in[0.0, 1.0]
      const int row    = std::min<int>(
          rs->rowCount - 1,
          std::max<int>(0, std::floor(proj_y * rs->rowCount)));

      const int col = std::min<int>(
          rs->columnCount - 1,
          std::max<int>(
              0, rs->columnCount * (yaw + M_PIf) / (2 * M_PIf)));

      rs->rangeImage(row, col)      = range_xy / rs->rangeResolution;
      rs->intensityImage(row, col)  = ptInt * 255;
      rs->organizedPoints(row, col) = {xs[i], ys[i], zs[i]};
    }

    // save:
    o = std::dynamic_pointer_cast<mrpt::obs::CObservation>(rs);
  }

  // Store in the output queue:
  read_ahead_lidar_obs_[step] = std::move(o);
}

constexpr size_t MAX_UNLOAD_LEN = 250;

void KittiOdometryDataset::AutoUnloadOldEntries() const {
  while (read_ahead_lidar_obs_.size() > MAX_UNLOAD_LEN)
      read_ahead_lidar_obs_.erase(read_ahead_lidar_obs_.begin());

  while (read_ahead_image_obs_.size() > MAX_UNLOAD_LEN)
      read_ahead_image_obs_.erase(read_ahead_image_obs_.begin());
}

}