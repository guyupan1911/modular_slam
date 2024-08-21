cmake_minimum_required(VERSION 3.5)

project(modular_slam_interface LANGUAGES CXX)

set(LIB_PUBLIC_HDRS
  hello_world.h
)

set(LIB_SRCS
  hello_world.cc
)

include_directories(${CMAKE_SOURCE_DIR})

add_library(hello_world_lib ${LIB_PUBLIC_HDRS} ${LIB_SRCS})

add_executable(hello_world_bin hello_world_cli.cc)
target_link_libraries(hello_world_bin hello_world_lib)