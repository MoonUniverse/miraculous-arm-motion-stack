cmake_minimum_required(VERSION 3.10)
project(arm_kinematics)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

foreach(target fk_demo ik_demo cartesian_path_demo)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    geometry_msgs
    moveit_ros_planning_interface
    rclcpp
  )
  install(TARGETS ${target}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

ament_package()
