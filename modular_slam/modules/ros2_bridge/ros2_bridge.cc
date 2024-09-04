/**
 * @file ros2_bridge.cc
 * @author your name (you@domain.com)
 * @brief convert to ros2 messages and publish them 
 * @version 0.1
 * @date 2024-09-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "modules/ros2_bridge/ros2_bridge.h"

#include <mrpt/ros1bridge/time.h>
#include <mrpt/ros1bridge/image.h>


namespace modular_slam {

void ROS2Bridge::Initialize() {
  ros_node_thread_ = std::thread(&ROS2Bridge::ROSNodeThreadMain, this);
  try {
    rclcpp::shutdown();
    if (ros_node_thread_.joinable()) {
      ros_node_thread_.detach();
    }
  } catch (const std::exception& e) {
      std::cerr << "[ROS2Bridge] Exception in Initialize:\n" << e.what();
  }
}

void ROS2Bridge::ROSNodeThreadMain() {
  const char* NODE_NAME = "modular_slam_ros2_bridge";
  
  try {
    const int argc = 1;
    char const* const argv[2] = {NODE_NAME, nullptr};
    
    if (!rclcpp::ok()) {
      rclcpp::init(argc, argv);
    }

    ros_node_ = std::make_shared<rclcpp::Node>(NODE_NAME);

    // spin
    rclcpp::spin(ros_node_);
    rclcpp::shutdown();
  } catch (const std::exception& e) {
    std::cerr << "[ROS2Bridge] Exception in ROSNodeThreadMain:\n" << e.what();
  }
}

ros::Time ROS2Bridge::MyNow(const mrpt::Clock::time_point& observationStamp) {
  if (publish_in_sim_time_) {
    return mrpt::ros1bridge::toROS(observationStamp);
  }
  else {
    return mrpt::ros1bridge::toROS(mrpt::Clock::now());
  }
}

void ROS2Bridge::OnNewObservation(const mrpt::obs::CObservation::Ptr& o) {
  
  ASSERT_(o);

  if (auto oImg = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(o); oImg) {
    return InternalOn(*oImg);
  }
}

void ROS2Bridge::InternalOn(const mrpt::obs::CObservationImage& obs) {
  auto lock = mrpt::lockHelper(ros_publish_mtx_);

  const bool is_first_pub = ros_pubs_.sensor_pubs.find(obs.sensorLabel) ==
                            ros_pubs_.sensor_pubs.end();
  
  auto& pub = ros_pubs_.sensor_pubs[obs.sensorLabel];
  
  if (is_first_pub) {
    pub = RosNode()->create_publisher<sensor_msgs::msg::Image>(
        obs.sensorLabel, rclcpp::SystemDefaultsQoS());
  }
  lock.unlock();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubImg =
      std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::Image>>(pub);
  ASSERT_(pubImg);

  const std::string sensor_frame_id = obs.sensorLabel;

  // send observation
  {
    obs.load();

    // Convert observation MRPT -> ROS1 -> ROS2
    sensor_msgs::Image msg_img;
    std_msgs::Header   msg_header;
    msg_header.stamp    = MyNow(obs.timestamp);
    msg_header.frame_id = sensor_frame_id;
    msg_img = mrpt::ros1bridge::toROS(obs.image, msg_header);

    sensor_msgs::msg::Image msg_img_ros2;
    std_msgs::msg::Header   msg_header_ros2;
    msg_header_ros2.stamp.sec    = msg_header.stamp.sec;
    msg_header_ros2.stamp.nanosec    = msg_header.stamp.nsec;
    msg_header_ros2.frame_id = msg_header.frame_id;

    msg_img_ros2.header = msg_header_ros2;
    msg_img_ros2.height = msg_img.height;
    msg_img_ros2.width = msg_img.width;
    msg_img_ros2.encoding = msg_img.encoding;
    msg_img_ros2.is_bigendian = msg_img.is_bigendian;
    msg_img_ros2.step = msg_img.step;
    msg_img_ros2.data = msg_img.data;

    pubImg->publish(msg_img_ros2);
  }
}


}