cmake_minimum_required(VERSION 3.5)

project(modular_slam_yaml LANGUAGES CXX)

# find dependencies:
find_package(MRPT 2.1.0 REQUIRED COMPONENTS containers system)

set(LIB_PUBLIC_HDRS
  yaml_helper.h
)

set(LIB_SRCS
)

include_directories(${CMAKE_SOURCE_DIR})

add_library(${PROJECT_NAME} ${LIB_PUBLIC_HDRS} ${LIB_SRCS})

target_link_libraries(${PROJECT_NAME}
  mrpt::containers
  mrpt::system
)