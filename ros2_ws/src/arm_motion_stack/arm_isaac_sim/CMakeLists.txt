cmake_minimum_required(VERSION 3.10)
project(arm_isaac_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS
  scripts/arm_isaac_ros_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
