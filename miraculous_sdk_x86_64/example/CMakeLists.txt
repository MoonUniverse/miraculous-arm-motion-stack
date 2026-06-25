#
# CMakeLists.txt — 示例程序独立编译配置
#
# SDK 目录下, 用户可直接编译示例:
#
#   cd <sdk_dir>/example
#   mkdir build && cd build
#   cmake ..
#   make
#
# 或指定 SDK 路径:
#   cmake .. -DSDK_PATH=/path/to/sdk
#
cmake_minimum_required(VERSION 3.10)
project(miraculous_sdk_examples LANGUAGES C)

# 默认 SDK_PATH 为上一级目录 (SDK 根目录, 包含 include/ 和 lib/)
if(NOT DEFINED SDK_PATH)
    set(SDK_PATH "..")
    message(STATUS "SDK_PATH not set, using default: ${SDK_PATH}")
endif()

include_directories(${SDK_PATH}/include)
link_directories(${SDK_PATH}/lib)

# 链接动态库的示例
function(add_so_example name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} miraculous_sdk pthread rt)
endfunction()

# 链接静态库的示例
function(add_a_example name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} miraculous_sdk.a pthread rt)
endfunction()

# === 所有示例 ===
add_so_example(example_pp_mode         test_pp_mode.c)
add_so_example(example_pv_mode         test_pv_mode.c)
add_so_example(example_pt_mode         test_pt_mode.c)
add_so_example(example_csp_mode        test_csp_mode.c)
add_so_example(example_csp_ex           test_csp_ex.c)
add_so_example(example_csv_mode        test_csv_mode.c)
add_so_example(example_cst_mode        test_cst_mode.c)
add_so_example(example_mit_mode        test_mit_mode.c)
add_so_example(example_loop_pp_mode    test_loop_pp_mode.c)
add_so_example(example_raw_can         test_raw_can.c)
add_so_example(example_pdo_config      test_pdo_config.c)
add_so_example(example_multi_motor     test_multi_motor.c)
add_so_example(example_read_params     test_read_params.c)
add_so_example(example_emcy_cb         test_emcy_callback.c)
add_so_example(example_heartbeat_cb    test_heartbeat_callback.c)
add_so_example(example_sdo_debug       test_sdo_debug.c)
add_so_example(example_get_position     test_get_position.c)
add_so_example(example_set_zero         test_set_zero.c)
