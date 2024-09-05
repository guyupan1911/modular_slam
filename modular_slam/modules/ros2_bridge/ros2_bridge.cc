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

#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/ros1bridge/time.h>
#include <mrpt/ros1bridge/image.h>
#include <mrpt/ros1bridge/point_cloud2.h>
#include <mrpt/ros1bridge/pose.h>

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

    auto lock_node = mrpt::lockHelper(ros_node_mtx_);
    ros_node_ = std::make_shared<rclcpp::Node>(NODE_NAME);
    lock_node.unlock();

    // tf broadcaster
    tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);

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
  } else if ( auto oPc = std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(o); oPc) {
    return InternalOn(*oPc);
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

void ROS2Bridge::InternalOn(const mrpt::obs::CObservationPointCloud& obs) {
  auto lock = mrpt::lockHelper(ros_publish_mtx_);

  const std::string label_points = obs.sensorLabel + "_points";

  const bool is_first_pub = ros_pubs_.sensor_pubs.find(label_points) ==
                            ros_pubs_.sensor_pubs.end();
  
  auto& pub = ros_pubs_.sensor_pubs[label_points];
  
  if (is_first_pub) {
    pub = RosNode()->create_publisher<sensor_msgs::msg::PointCloud2>(
        label_points, rclcpp::SensorDataQoS());
  }
  lock.unlock();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPoints =
      std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>(pub);
  ASSERT_(pubPoints);

  const std::string sensor_frame_id = label_points;

  // send tf
  {
    mrpt::poses::CPose3D sensorPose = obs.sensorPose;

    tf2::Transform transform = mrpt::ros1bridge::toROS_tfTransform(sensorPose);

    geometry_msgs::msg::TransformStamped tfStmp;
    // tfStmp.transform       = tf2::toMsg(transform);
    tfStmp.child_frame_id  = sensor_frame_id;
    tfStmp.header.frame_id = "base_link";
    tfStmp.header.stamp.sec    = MyNow(obs.timestamp).sec;
    tfStmp.header.stamp.nanosec    = MyNow(obs.timestamp).nsec;
    tf_bc_->sendTransform(tfStmp);
  }

  // send observation
  if (obs.pointcloud) {
    // Convert observation MRPT -> ROS1 -> ROS2
    sensor_msgs::PointCloud2 msg_pts;
    std_msgs::Header   msg_header;
    msg_header.stamp    = MyNow(obs.timestamp);
    msg_header.frame_id = sensor_frame_id;

    obs.load();

    int field_size = 0;
    if (auto* xyzirt = dynamic_cast<const mrpt::maps::CPointsMapXYZIRT*>(
        obs.pointcloud.get()); xyzirt) {
        mrpt::ros1bridge::toROS(*xyzirt, msg_header, msg_pts);
        field_size = 3;
    } else if (auto* xyzi = dynamic_cast<const mrpt::maps::CPointsMapXYZI*>(
        obs.pointcloud.get()); xyzi) {
        mrpt::ros1bridge::toROS(*xyzi, msg_header, msg_pts);
        field_size = 4;
    } else if (auto* sPts = dynamic_cast<const mrpt::maps::CSimplePointsMap*>(
        obs.pointcloud.get()); sPts) {
        mrpt::ros1bridge::toROS(*sPts, msg_header, msg_pts);
        field_size = 3;
    } else {
      THROW_EXCEPTION_FMT(
        "Do not know how to handle this variant of CPointsMap: "
        "class='%s'",
        obs.pointcloud->GetRuntimeClass()->className);
    }

    sensor_msgs::msg::PointCloud2 msg_pts_ros2;
    msg_pts_ros2.header.stamp.sec =  msg_header.stamp.sec;
    msg_pts_ros2.header.stamp.nanosec =  msg_header.stamp.nsec;
    // msg_pts_ros2.header.frame_id =  msg_header.frame_id;
    msg_pts_ros2.header.frame_id = sensor_frame_id;

    msg_pts_ros2.height = msg_pts.height;
    msg_pts_ros2.width = msg_pts.width;
    msg_pts_ros2.fields.resize(field_size);
    for (int i = 0; i < field_size; ++i) {
      msg_pts_ros2.fields[i].name = msg_pts.fields[i].name;
      msg_pts_ros2.fields[i].offset = msg_pts.fields[i].offset;
      msg_pts_ros2.fields[i].datatype = msg_pts.fields[i].datatype;
      msg_pts_ros2.fields[i].count = msg_pts.fields[i].count;
    }
    msg_pts_ros2.is_bigendian = msg_pts.is_bigendian;
    msg_pts_ros2.point_step = msg_pts.point_step;
    msg_pts_ros2.row_step = msg_pts.row_step;
    msg_pts_ros2.data = msg_pts.data;
    msg_pts_ros2.is_dense = msg_pts.is_dense;

    pubPoints->publish(msg_pts_ros2);
  }  
}

}