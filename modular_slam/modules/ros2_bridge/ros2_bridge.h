/**
 * @file ros2_bridge.h
 * @author your name (you@domain.com)
 * @brief convert to ros2 messages and publish them 
 * @version 0.1
 * @date 2024-09-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ros/time.h>

// #include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>

#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationPointCloud.h>

#include "modules/interface/raw_data_consumer.h"

namespace modular_slam {

class ROS2Bridge : public RawDataConsumer {
 public:
  ROS2Bridge() = default;
  ~ROS2Bridge() = default;

  void Initialize();

  void OnNewObservation(const mrpt::obs::CObservation::Ptr& o) override;

 private:
  void ROSNodeThreadMain();

  ros::Time MyNow(const mrpt::Clock::time_point& observationStamp);

  std::shared_ptr<rclcpp::Node> RosNode() {
    auto lck = mrpt::lockHelper(ros_node_mtx_);
    return ros_node_;
  }

  void InternalOn(const mrpt::obs::CObservationImage& obs);
  void InternalOn(const mrpt::obs::CObservationPointCloud& obs);

 private:
  // parameters
  bool publish_in_sim_time_ = false;

  // ros
  std::shared_ptr<rclcpp::Node> ros_node_;
  std::mutex ros_node_mtx_;
  std::thread ros_node_thread_;
  std::mutex ros_publish_mtx_;

  // tf
  // std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  // std::shared_ptr<tf2::BufferCore>            tf_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<tf2_ros::TransformBroadcaster>       tf_bc_;
  // std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_bc_;

  struct RosPubs {
    std::map<std::string, rclcpp::PublisherBase::SharedPtr> sensor_pubs;
  };

  RosPubs ros_pubs_;
};

}
