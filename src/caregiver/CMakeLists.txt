cmake_minimum_required(VERSION 3.8)
project(caregiver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Executables

# CaregiverPoseNode
add_executable(caregiver_pose_node
  src/publish_caregiver_pos.cpp
)
ament_target_dependencies(caregiver_pose_node
  rclcpp
  nav_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ModeManager: f/s
add_executable(mode_manager
  src/mode_manager.cpp
)
ament_target_dependencies(mode_manager
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  nav2_msgs
)

# Install
install(
  TARGETS
    caregiver_pose_node
    mode_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
