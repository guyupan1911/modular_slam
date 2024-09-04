cmake_minimum_required(VERSION 3.5)

project(modular_slam_ros2_bridge LANGUAGES CXX)

find_package(mrpt-ros1bridge)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(roscpp REQUIRED)


set(LIB_PUBLIC_HDRS
  ros2_bridge.h
)

set(LIB_SRCS
  ros2_bridge.cc
)

include_directories(${CMAKE_SOURCE_DIR})

add_library(${PROJECT_NAME} ${LIB_PUBLIC_HDRS} ${LIB_SRCS})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
)

target_link_libraries(${PROJECT_NAME}
  modular_slam_interface
  mrpt::ros1bridge
)
