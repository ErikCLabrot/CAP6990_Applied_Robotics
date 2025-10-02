cmake_minimum_required(VERSION 3.10)
project(graph_slam_solver)

find_package(Ceres REQUIRED)

add_executable(graph_slam_solver graph_slam_solver.cpp)
target_link_libraries(graph_slam_solver PRIVATE Ceres::ceres)