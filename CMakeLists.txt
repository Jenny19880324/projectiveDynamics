cmake_minimum_required(VERSION 2.8.9)
project(projectiveDynamics)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(projectiveDynamics src/main.cpp)
