# SDK 构建与链接指南

## 快速构建

```bash
# 克隆/进入 SDK 目录
cd miraculous_SDK
mkdir build && cd build

# 配置
cmake ..

# 编译
make -j$(nproc)

# 查看编译产物
ls lib/     # libmiraculous_sdk.a, libmiraculous_sdk_shared.so
ls bin/     # example_* 可执行文件
```

## CMake 选项

| 选项 | 默认值 | 说明 |
| --- | --- | --- |
| `CMAKE_BUILD_TYPE` | (空) | `Debug` 或 `Release` |
| `CMAKE_INSTALL_PREFIX` | `/usr/local` | 安装路径前缀 |

## 链接到自己的项目

### 方式 1：直接链接 .so 动态库

```bash
gcc -o my_app my_app.c \
    -I/path/to/miraculous_SDK/include \
    -L/path/to/miraculous_SDK/lib \
    -lmiraculous_sdk_shared \
    -lpthread -lrt
```

运行前设置库路径：

```bash
export LD_LIBRARY_PATH=/path/to/miraculous_SDK/lib:$LD_LIBRARY_PATH
./my_app
```

### 方式 2：直接链接 .a 静态库

```bash
gcc -o my_app my_app.c \
    -I/path/to/miraculous_SDK/include \
    /path/to/miraculous_SDK/lib/libmiraculous_sdk.a \
    -lpthread -lrt
```

静态链接无需运行时依赖 `.so`。

### 方式 3：CMake 集成

在 `CMakeLists.txt` 中：

```cmake
# 指定 SDK 路径
set(SDK_PATH "/path/to/miraculous_SDK")

# 头文件
include_directories(${SDK_PATH}/include)

# 库路径
link_directories(${SDK_PATH}/lib)

# 链接动态库
add_executable(my_app my_app.c)
target_link_libraries(my_app miraculous_sdk_shared pthread rt)
```

或参考 `examples/CMakeLists_examples.txt` 获取完整示例。

### 方式 4：安装到系统路径

```bash
# 构建后安装
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make install

# 然后直接编译
gcc -o my_app my_app.c -lmiraculous_sdk_shared -lpthread -lrt
```

## 运行时库依赖

- `libpthread.so` — POSIX 线程
- `librt.so` — POSIX 实时扩展（timerfd 等）

## 头文件

对外只暴露一个头文件：

```c
#include "miraculous_sdk.h"
```

无需 `#include` 任何其他 SDK 头文件。

## 平台要求

- Linux 内核 >= 3.6（CAN_RAW socket 支持）
- CAN 接口已启用（如 `can0`）
- GCC >= 4.8 或 Clang
- CMake >= 3.10
