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
#include <mrpt/obs/CObservationImage.h>


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

size_t KittiOdometryDataset::DatasetSize() const { return {}; }

mrpt::obs::CSensoryFrame::Ptr KittiOdometryDataset::GetObservation(size_t timestamp) const {
  return {};
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

  
}
 
}
