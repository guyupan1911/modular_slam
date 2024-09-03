/**
 * @file kitti_odometry_dataset_cli.cc
 * @author guyupan
 * @brief kitti odometry dataset client
 * @version 0.1
 * @date 2024-09-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "modules/input_kitti_dataset/kitti_odometry_dataset.h"

using modular_slam::KittiOdometryDataset;

int main() {
  auto dataset = std::make_shared<KittiOdometryDataset>();
  dataset->setMinLoggingLevel(mrpt::system::VerbosityLevel::LVL_DEBUG);

  std::string yaml_filename = "/modular_slam_ws/src/modular_slam/modules/input_kitti_dataset/config/kitti_odometry.yaml";
  const auto cfg = mrpt::containers::yaml::FromFile(yaml_filename);

  dataset->Initialize_rds(cfg);

}