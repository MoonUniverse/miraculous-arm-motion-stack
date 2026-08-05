cmake_minimum_required(VERSION 3.10)
project(arm_remote_control)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(remote_pc_heartbeat src/remote_pc_heartbeat.cpp)
ament_target_dependencies(remote_pc_heartbeat rclcpp std_msgs)

add_executable(remote_motion_watchdog src/remote_motion_watchdog.cpp)
ament_target_dependencies(
  remote_motion_watchdog
  control_msgs
  controller_manager_msgs
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
)

install(
  TARGETS remote_pc_heartbeat remote_motion_watchdog
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
