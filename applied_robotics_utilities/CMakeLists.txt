cmake_minimum_required(VERSION 3.8)
project(applied_robotics_utilities)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/LandmarkObservation.msg
  msg/LandmarkArray.msg
  action/MoveToGoal.action
  action/SaveImage.action
  srv/CESPathOptimize.srv
  srv/CESOptimizeConfigure.srv
  srv/MoveStateCancel.srv
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
