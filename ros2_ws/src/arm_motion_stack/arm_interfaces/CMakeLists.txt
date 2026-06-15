cmake_minimum_required(VERSION 3.10)
project(arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeIK.srv"
  "srv/ComputeFK.srv"
  "msg/JointTrajectoryStatus.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_package()
