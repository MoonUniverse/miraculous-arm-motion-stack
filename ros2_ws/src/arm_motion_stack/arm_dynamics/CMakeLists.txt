cmake_minimum_required(VERSION 3.10)
project(arm_dynamics)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)

add_library(dynamics_model src/dynamics_model.cpp)
target_include_directories(dynamics_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dynamics_model Eigen3::Eigen)

add_executable(dynamics_demo src/dynamics_demo.cpp)
target_link_libraries(dynamics_demo dynamics_model)
ament_target_dependencies(dynamics_demo rclcpp)

install(TARGETS dynamics_model dynamics_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
