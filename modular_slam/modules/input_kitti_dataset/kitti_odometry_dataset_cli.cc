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

#include <iostream>

#include <mrpt/obs/CObservationPointCloud.h>

#include "modules/input_kitti_dataset/kitti_odometry_dataset.h"
#include "modules/ros2_bridge/ros2_bridge.h"

using modular_slam::KittiOdometryDataset;
using modular_slam::ROS2Bridge;

int main() {

  // ros2 bridge
  auto ros_node = std::make_shared<ROS2Bridge>();
  ros_node->Initialize();
  
  // kitti dataset
  auto dataset = std::make_shared<KittiOdometryDataset>();
  dataset->setMinLoggingLevel(mrpt::system::VerbosityLevel::LVL_DEBUG);

  std::string yaml_filename = "/modular_slam_ws/src/modular_slam/modules/input_kitti_dataset/config/kitti_odometry.yaml";
  const auto cfg = mrpt::containers::yaml::FromFile(yaml_filename);

  dataset->Initialize_rds(cfg);

  size_t last_dataset_entry = dataset->DatasetSize();
  size_t first_dataset_entry = 0;

  // Run:
  for (size_t i = first_dataset_entry; i < last_dataset_entry; ++i) {
    // Get observations from the dataset:
    using mrpt::obs::CObservation2DRangeScan;
    using mrpt::obs::CObservation3DRangeScan;
    using mrpt::obs::CObservationGPS;
    using mrpt::obs::CObservationOdometry;
    using mrpt::obs::CObservationPointCloud;
    using mrpt::obs::CObservationRotatingScan;
    using mrpt::obs::CObservationVelodyneScan;
    using mrpt::obs::CObservationImage;

    const auto sf = dataset->GetObservations(i);
    ASSERT_(sf);

    std::cout << "entry " << i << " : \n";
    for (auto iter = sf->begin(); iter != sf->end(); ++iter) {
      std::cout << (*iter)->sensorLabel << " ";
    }
    std::cout << "\n";

    mrpt::obs::CObservation::Ptr obs;
    if (obs = sf->getObservationByClass<CObservationImage>(), obs) {
      ros_node->OnNewObservation(obs);
    }
    if (obs = sf->getObservationByClass<CObservationPointCloud>(); obs) {
      ros_node->OnNewObservation(obs);
    }
    // if (!obs) obs = sf->getObservationByClass<CObservation3DRangeScan>();
    // if (!obs) obs = sf->getObservationByClass<CObservation2DRangeScan>();
    // if (!obs) obs = sf->getObservationByClass<CObservationVelodyneScan>();
    // if (!obs) obs = sf->getObservationByClass<CObservationGPS>();
    // if (!obs) obs = sf->getObservationByClass<CObservationOdometry>();

    // if (!obs) continue;
  }

}