cmake_minimum_required(VERSION 3.5)

project(modular_slam_kitti_odometry_dataset LANGUAGES CXX)

find_package(mrpt-obs)
find_package(mrpt-poses)

set(LIB_PUBLIC_HDRS
  kitti_odometry_dataset.h
)

set(LIB_SRCS
  kitti_odometry_dataset.cc
)

include_directories(${CMAKE_SOURCE_DIR})

add_library(${PROJECT_NAME} ${LIB_PUBLIC_HDRS} ${LIB_SRCS})

target_link_libraries(${PROJECT_NAME}
  mrpt::obs
  mrpt::poses
)

add_executable(kitti_odometry_dataset_cli kitti_odometry_dataset_cli.cc)
target_link_libraries(kitti_odometry_dataset_cli
  ${PROJECT_NAME}
  modular_slam_interface
)