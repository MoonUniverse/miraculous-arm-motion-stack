cmake_minimum_required(VERSION 3.10)
project(miraculous_driver)

# ---- miraculous_sdk (CANopen/CiA402 motor C SDK) ----
# The SDK lives outside the ROS workspace at <project_root>/miraculous_sdk.
# From ros2_ws/src/arm_motion_stack/miraculous_driver that is 4 levels up.
#
# Two SDK variants may exist side-by-side:
#   miraculous_sdk          – ARM aarch64 (target device)
#   miraculous_sdk_x86_64    – x86-64 (development host)
# We auto-select based on the host architecture so the package builds on both.
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "amd64")
  set(_SDK_DEFAULT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../miraculous_sdk_x86_64")
else()
  set(_SDK_DEFAULT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../miraculous_sdk")
endif()

set(MIRACULOUS_SDK_DIR
  "${_SDK_DEFAULT}"
  CACHE PATH "Path to the miraculous_sdk root (contains include/ and lib/)")

if(NOT EXISTS "${MIRACULOUS_SDK_DIR}/include/miraculous_sdk.h")
  message(FATAL_ERROR
    "miraculous_sdk header not found at '${MIRACULOUS_SDK_DIR}/include/miraculous_sdk.h'. "
    "Set -DMIRACULOUS_SDK_DIR=/abs/path/to/miraculous_sdk")
endif()

find_library(MIRACULOUS_SDK_LIB
  NAMES miraculous_sdk
  PATHS "${MIRACULOUS_SDK_DIR}/lib"
  REQUIRED)
message(STATUS "miraculous_sdk: include=${MIRACULOUS_SDK_DIR}/include lib=${MIRACULOUS_SDK_LIB}")

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ---- Core + plugin shared library ----
add_library(${PROJECT_NAME} SHARED
  src/miraculous_arm.cpp
  src/miraculous_system.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${MIRACULOUS_SDK_DIR}/include
)
target_link_directories(${PROJECT_NAME} PRIVATE ${MIRACULOUS_SDK_DIR}/lib)
target_link_libraries(${PROJECT_NAME}
  ${MIRACULOUS_SDK_LIB}
  pthread
  rt
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)
# Runtime: find libmiraculous_sdk.so next to this library ($ORIGIN).
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${MIRACULOUS_SDK_DIR}/lib"
  INSTALL_RPATH "$ORIGIN"
)
message(STATUS "Host arch: ${CMAKE_SYSTEM_PROCESSOR}  =>  SDK: ${MIRACULOUS_SDK_DIR}")

# ---- teach_record / playback executables ----
add_executable(teach_record_node src/teach_record_node.cpp)
target_include_directories(teach_record_node PRIVATE ${MIRACULOUS_SDK_DIR}/include)
target_link_libraries(teach_record_node ${PROJECT_NAME})
ament_target_dependencies(teach_record_node rclcpp sensor_msgs std_srvs)
set_target_properties(teach_record_node PROPERTIES INSTALL_RPATH "$ORIGIN")

add_executable(playback_node src/playback_node.cpp)
target_include_directories(playback_node PRIVATE ${MIRACULOUS_SDK_DIR}/include)
target_link_libraries(playback_node ${PROJECT_NAME})
ament_target_dependencies(playback_node rclcpp sensor_msgs std_srvs)
set_target_properties(playback_node PROPERTIES INSTALL_RPATH "$ORIGIN")

# ---- trajectory tracking test executable ----
add_executable(trajectory_tracking_test_node src/trajectory_tracking_test_node.cpp)
target_include_directories(trajectory_tracking_test_node PRIVATE ${MIRACULOUS_SDK_DIR}/include)
target_link_libraries(trajectory_tracking_test_node ${PROJECT_NAME})
ament_target_dependencies(trajectory_tracking_test_node rclcpp sensor_msgs std_srvs)
set_target_properties(trajectory_tracking_test_node PROPERTIES INSTALL_RPATH "$ORIGIN")

# ---- pluginlib export ----
pluginlib_export_plugin_description_file(hardware_interface miraculous_driver_plugins.xml)

# ---- install ----
# Shared library + plugin: install to lib/ (pluginlib looks here).
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# Executables: install to lib/<pkg> (ros2 run expects them here).
install(
  TARGETS teach_record_node playback_node trajectory_tracking_test_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Ship the prebuilt SDK shared library next to our targets so $ORIGIN RPATH resolves.
install(FILES "${MIRACULOUS_SDK_DIR}/lib/libmiraculous_sdk.so" DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/plot_trajectory.py DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_srvs
)

ament_package()
