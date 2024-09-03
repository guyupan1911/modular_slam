cmake_minimum_required(VERSION 3.5)

project(modular_slam LANGUAGES CXX)

add_subdirectory(modules/interface)
add_subdirectory(modules/input_kitti_dataset)
# add_subdirectory(modules/yaml)