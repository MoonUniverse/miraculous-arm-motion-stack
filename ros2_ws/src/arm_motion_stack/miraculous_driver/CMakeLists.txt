cmake_minimum_required(VERSION 3.10)
project(miraculous_driver)

# ---- miraculous_sdk (CANopen/CiA402 motor C SDK) ----
# The SDK is owned and built by the motor team; this package only consumes
# include/miraculous_sdk.h plus a built libmiraculous_sdk. Both layouts work:
#   - source checkout with build artifacts in build/lib/ (cmake build)
#   - delivered/installed prefix with lib/
# Default is the checkout at <project_root>/miraculous_sdk. Override with:
#   -DMIRACULOUS_SDK_DIR=/abs/path/to/sdk
set(MIRACULOUS_SDK_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../miraculous_sdk"
  CACHE PATH "Path to the miraculous_sdk root (source checkout or installed prefix)")

if(NOT EXISTS "${MIRACULOUS_SDK_DIR}/include/miraculous_sdk.h")
  message(FATAL_ERROR
    "miraculous_sdk header not found at '${MIRACULOUS_SDK_DIR}/include/miraculous_sdk.h'. "
    "Set -DMIRACULOUS_SDK_DIR=/abs/path/to/sdk")
endif()

find_library(MIRACULOUS_SDK_LIB
  NAMES miraculous_sdk
  PATHS "${MIRACULOUS_SDK_DIR}/build/lib" "${MIRACULOUS_SDK_DIR}/lib"
  NO_DEFAULT_PATH)
if(NOT MIRACULOUS_SDK_LIB)
  message(FATAL_ERROR
    "libmiraculous_sdk not found under '${MIRACULOUS_SDK_DIR}/build/lib' or "
    "'${MIRACULOUS_SDK_DIR}/lib'. Get a built SDK from the motor team, or build "
    "the checkout once:\n"
    "  cmake -S ${MIRACULOUS_SDK_DIR} -B ${MIRACULOUS_SDK_DIR}/build\n"
    "  cmake --build ${MIRACULOUS_SDK_DIR}/build")
endif()
get_filename_component(MIRACULOUS_SDK_LIBDIR "${MIRACULOUS_SDK_LIB}" DIRECTORY)
message(STATUS "miraculous_sdk: include=${MIRACULOUS_SDK_DIR}/include lib=${MIRACULOUS_SDK_LIB}")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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
target_link_directories(${PROJECT_NAME} PRIVATE ${MIRACULOUS_SDK_LIBDIR})
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
  std_msgs
)
# Runtime: find libmiraculous_sdk.so next to this library ($ORIGIN).
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${MIRACULOUS_SDK_LIBDIR}"
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

# ---- tests -----------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_enable_csp
    test/test_enable_csp.cpp
    src/miraculous_arm.cpp
  )
  target_include_directories(test_enable_csp
    PRIVATE
      include
      ${MIRACULOUS_SDK_DIR}/include
  )
  target_link_libraries(test_enable_csp
    ${MIRACULOUS_SDK_LIB}
    pthread
    rt
  )
  set_target_properties(test_enable_csp PROPERTIES
    BUILD_RPATH "${MIRACULOUS_SDK_LIBDIR}"
    LINK_FLAGS
      "-Wl,--wrap=miraculous_motor_sync_stop \
       -Wl,--wrap=miraculous_motor_sync_start \
       -Wl,--wrap=miraculous_motor_sync_send \
       -Wl,--wrap=miraculous_motor_get_state \
       -Wl,--wrap=miraculous_motor_shutdown \
       -Wl,--wrap=miraculous_motor_wait_state \
       -Wl,--wrap=miraculous_motor_get_mode \
       -Wl,--wrap=miraculous_motor_set_mode \
       -Wl,--wrap=miraculous_motor_switch_on \
       -Wl,--wrap=miraculous_motor_get_position_ex \
       -Wl,--wrap=miraculous_motor_get_velocity_ex \
       -Wl,--wrap=miraculous_motor_csp_set_target_ex \
       -Wl,--wrap=miraculous_motor_enable \
       -Wl,--wrap=miraculous_motor_disable \
       -Wl,--wrap=miraculous_motor_quick_stop \
       -Wl,--wrap=miraculous_motor_disable_voltage \
       -Wl,--wrap=miraculous_motor_fault_reset"
  )

  ament_add_gtest(test_playback_csv_header
    test/test_playback_csv_header.cpp
  )
  target_include_directories(test_playback_csv_header PRIVATE src)

  ament_add_gtest(test_strict_parameter_lists
    test/test_strict_parameter_lists.cpp
  )
  target_include_directories(test_strict_parameter_lists PRIVATE src)

  ament_add_gtest(test_miraculous_system
    test/test_miraculous_system.cpp
  )
  target_include_directories(test_miraculous_system
    PRIVATE
      include
      ${MIRACULOUS_SDK_DIR}/include
  )
  target_link_libraries(test_miraculous_system ${PROJECT_NAME})
  ament_target_dependencies(test_miraculous_system
    hardware_interface
    rclcpp
    rclcpp_lifecycle
  )
endif()

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

# Ship the SDK shared library next to our targets so $ORIGIN RPATH resolves.
# (When find_library picked the static .a, nothing needs installing.)
if(MIRACULOUS_SDK_LIB MATCHES "\\.so$")
  install(FILES "${MIRACULOUS_SDK_LIB}" DESTINATION lib)
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config docs launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/plot_trajectory.py DESTINATION lib/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR python/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  std_srvs
)

ament_package()
