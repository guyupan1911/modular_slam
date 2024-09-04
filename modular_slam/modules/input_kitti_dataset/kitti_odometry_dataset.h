/**
 * @file kitti_odometry_dataset.h
 * @author guyupan
 * @brief kitti odometrty dataset
 * @version 0.1
 * @date 2024-09-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include "mrpt/system/COutputLogger.h"
#include <mrpt/img/TCamera.h>


#include "modules/interface/offline_dataset_source.h"
#include "modules/interface/raw_datasource_base.h"

namespace modular_slam {

class KittiOdometryDataset : public RawDataSourceBase,
                             public OfflineDatasetSource,
                             public mrpt::system::COutputLogger {

 public:
  KittiOdometryDataset();
  ~KittiOdometryDataset() override = default;

  // void SpinOnce();

  bool HasGroundTruthTrajectory() const override {
    return !ground_truth_trajectory_.empty();
  }

  mrpt::poses::CPose3DInterpolator GetGroundTruthTrajectory() const override {
    return ground_truth_trajectory_;
  }

  void Initialize_rds(const mrpt::containers::yaml& yaml) override;

  size_t DatasetSize() const override;

  std::shared_ptr<mrpt::obs::CObservationImage> GetImage(
    const unsigned int cam_idx, size_t step) const;

  std::shared_ptr<mrpt::obs::CObservation> GetPointCloud(size_t step) const;

  mrpt::obs::CSensoryFrame::Ptr GetObservations(size_t timestamp) const override;

  /** See:
   *  "IMLS-SLAM: scan-to-model matching based on 3D data", JE Deschaud, 2018.
   */
  double VERTICAL_ANGLE_OFFSET = mrpt::DEG2RAD(0.205);

 private:
  void LoadImage(const unsigned int cam_idx, const size_t step) const;
  void LoadLidar(size_t step) const;

  void AutoUnloadOldEntries() const;

 private:
  bool initialized_ = false;
  /* yaml parameters*/
  std::string base_dir_;
  std::string sequence_;
  bool clouds_as_organized_points_ = false;
  unsigned int range_matrix_column_count_ = 2000;
  unsigned int range_matrix_row_count_ = 64;

  bool publish_lidar_{true};
  bool publish_ground_truth_{true};
  std::array<bool, 4> publish_image_{{true, true, true, true}};
  std::array<mrpt::img::TCamera, 4> cam_intrinsics_;
  std::array<mrpt::math::TPose3D, 4> cam_poses_;

  std::vector<double> lst_timestamps_;
  std::string seq_dir_;

  std::array<std::vector<std::string>, 4> lst_image_;
  std::vector<std::string> lst_velodyne_;
  mrpt::math::CMatrixDouble ground_truth_poses_;
  mrpt::poses::CPose3DInterpolator ground_truth_trajectory_;
  mutable std::map<size_t, mrpt::obs::CObservation::Ptr>
      read_ahead_lidar_obs_;
  mutable std::map<size_t, std::array<mrpt::obs::CObservation::Ptr, 4>>
      read_ahead_image_obs_;

};

}