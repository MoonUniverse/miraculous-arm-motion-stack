cmake_minimum_required(VERSION 3.10)
project(arm_planning_examples)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

foreach(target plan_to_joint_target plan_to_pose_target plan_cartesian_path execute_named_pose)
  add_executable(${target} src/${target}.cpp)
  target_include_directories(${target} PRIVATE include)
  ament_target_dependencies(${target}
    ament_index_cpp
    geometry_msgs
    moveit_ros_planning_interface
    rclcpp
  )
  install(TARGETS ${target}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
