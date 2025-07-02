cmake_minimum_required(VERSION 3.10)
project(TrajectoryFrenet)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_library(
  trajectory_frenet 
  src/utils.cpp
)

target_link_libraries(
  trajectory_frenet
  PUBLIC Eigen3::Eigen
)

target_include_directories(
  trajectory_frenet
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)