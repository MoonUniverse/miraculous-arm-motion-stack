cmake_minimum_required(VERSION 3.10)
project(miraculous_sdk VERSION 1.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# --- 版本信息 ---
set(SDK_VERSION "1.1.0")

# --- 编译选项 ---
add_compile_options(-Wall -Wextra -Wpedantic -O2 -fPIC)

# --- 包含路径 ---
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# --- 源文件 ---
set(SDK_SOURCES
    src/can_socket.c
    src/co_master.c
    src/co_nmt.c
    src/co_sdo.c
    src/co_pdo.c
    src/co_sync.c
    src/co_heartbeat.c
    src/co_emcy.c
    src/motion_state.c
    src/version.c
    src/motion_pp.c
    src/motion_pv.c
    src/motion_pt.c
    src/motion_csp.c
    src/motion_csv.c
    src/motion_cst.c
    src/motion_mit.c
    src/utils.c
)

# --- 静态库 ---
add_library(miraculous_sdk STATIC ${SDK_SOURCES})
set_target_properties(miraculous_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(miraculous_sdk pthread rt)
set_target_properties(miraculous_sdk PROPERTIES
    OUTPUT_NAME "miraculous_sdk"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

# --- 共享库 ---
add_library(miraculous_sdk_shared SHARED ${SDK_SOURCES})
target_link_libraries(miraculous_sdk_shared pthread rt)
set_target_properties(miraculous_sdk_shared PROPERTIES
    OUTPUT_NAME "miraculous_sdk"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

# --- 示例程序 (仅开发时编译) ---
option(BUILD_EXAMPLES "Build example programs" ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# --- 工具 ---
add_executable(motor_scan tools/motor_scan.c)
target_link_libraries(motor_scan miraculous_sdk)
set_target_properties(motor_scan PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_executable(motor_diag tools/motor_diag.c)
target_link_libraries(motor_diag miraculous_sdk)
set_target_properties(motor_diag PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_executable(motor_enc_calib tools/motor_enc_calib.c)
target_link_libraries(motor_enc_calib miraculous_sdk)
set_target_properties(motor_enc_calib PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# --- 测试 ---
add_executable(test_vcan tests/test_vcan.c)
target_link_libraries(test_vcan miraculous_sdk)
set_target_properties(test_vcan PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# =====================================================================
# Install 规则 — 输出结构
#
# 安装后的目录结构:
#   prefix/
#   ├── include/
#   │   └── miraculous_sdk.h
#   ├── lib/
#   │   ├── libmiraculous_sdk.a
#   │   └── libmiraculous_sdk.so
#   ├── example/
#   │   ├── CMakeLists.txt
#   │   └── test_*.c
#   └── doc/
#       └── html/           (Doxygen 文档, 需 make docs 生成)
# =====================================================================

# --- include ---
install(FILES include/miraculous_sdk.h
        DESTINATION include)

# --- lib ---
install(TARGETS miraculous_sdk miraculous_sdk_shared
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib)

# --- example ---
install(DIRECTORY examples/
        DESTINATION example
        FILES_MATCHING PATTERN "*.c"
        PATTERN "CMakeLists.txt" EXCLUDE)
install(FILES examples/CMakeLists.txt
        DESTINATION example)

# --- doc (可选: 需先安装 doxygen) ---
find_program(DOXYGEN doxygen)
if(DOXYGEN)
    add_custom_target(docs
        COMMAND ${DOXYGEN} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen"
    )
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc/html/
            DESTINATION doc/html
            OPTIONAL)
endif()
