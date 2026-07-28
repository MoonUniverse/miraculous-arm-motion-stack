# Miraculous SDK

miraculous 电机驱动 SDK，基于 Linux SocketCAN 实现 CANopen-CiA402 协议栈，提供完整的电机控制 API。

## 架构

### 分层设计

```
┌─────────────────────────────────────────────────────┐
│  用户代码                                             │
│  #include "miraculous_sdk.h"                         │
└──────────────────┬──────────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────────┐
│  CiA402 运动控制层 (motion_*.c)                      │
│  状态机: full_enable / shutdown / fault_reset        │
│  模式: PP / PV / PT / CSP / CSV / CST / MIT         │
│  TPDO 缓存: get_position / get_velocity / get_torque │
└──────────────────┬──────────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────────┐
│  CANopen 主站层 (co_*.c)                             │
│  NMT — 网络管理 (start/stop/reset)                    │
│  SDO — 服务数据对象 (同步/异步双模式)                    │
│  PDO — 过程数据对象 (RPDO配置/TPDO分发)                │
│  SYNC — 同步触发 (定时器/手动)                          │
│  EMCY — 紧急事件                                     │
│  Heartbeat — 心跳监测                                 │
└──────────────────┬──────────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────────┐
│  CAN 传输层 (can_socket.c)                           │
│  SocketCAN 封装 + 互斥锁 + epoll + CAN_RAW_FILTER   │
└──────────────────┬──────────────────────────────────┘
                   │
              ┌────▼────┐
              │  CAN 总线 │
              └─────────┘
```

### 线程模型

单总线单线程架构 — 每路 CAN 总线只有一个专用的 **接收线程** + 一个 socket fd，承载该总线所有节点的通信：

```
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│  App Thread 1│    │  App Thread 2│    │  App Thread 3│
│ (motor node1)│    │ (motor node2)│    │ (motor node5)│
└──────┬───────┘    └──────┬───────┘    └──────┬───────┘
       │                   │                   │
       │        ┌──────────▼──────────┐        │
       │        │  MiraCoMaster       │        │
       │        │  ┌───────────────┐  │        │
       │        │  │ 接收线程       │  │        │
       └────────┤  │ epoll_wait    │  ├────────┘
                │  │ → SDO dispatch│  │
                │  │ → TPDO cache  │  │
                │  │ → Heartbeat   │  │
                │  │ → EMCY        │  │
                │  └───────────────┘  │
                │  sdo_node_lock[128] │ ← 每节点独立锁
                │  motor_by_node[128]│ ← 电机索引表
                └────────────────────┘
                        │
               ┌────────▼────────┐
               │  SocketCAN fd   │
               │  (单 fd)        │
               └─────────────────┘
```

**线程安全设计：**
- **发送锁** — `can_socket.c` 的 `ctx->lock` 保护 `write()` 调用，防止多线程并发发包导致帧错乱
- **每节点 SDO 锁** — `co->sdo_node_lock[node_id]`，不同节点 SDO 可并行，同一节点串行
- **条件变量等待** — SDO 调用者通过 `pthread_cond_timedwait` 阻塞等待，接收线程通过 `pthread_cond_signal` 唤醒
- **TPDO 无锁读取** — TPDO 数据通过 recv 线程写入缓存，用户线程通过 `__sync_synchronize` 内存屏障安全读取
- **全局总线注册表** — `g_bus_lock` 保护多个电机 open/close 时的竞态

### SDO 协议 (co_sdo.c)

支持同步和异步双模式：

**同步 SDO：**
1. 设置 `sdo_node_busy[node_id] = true`，记录当前请求的 index/subindex
2. 发送 SDO 请求帧
3. 在 `sdo_node_cond[node_id]` 上等待（带超时）
4. 接收线程收到 SDO 响应 → 匹配 index/subindex → 填充数据 → signal
5. 调用者唤醒，读取数据

**异步 SDO：**
1. 创建 `MiraSdoAsyncTask` 链表节点，注册回调
2. 发送 SDO 请求，立即返回
3. 接收线程收到响应 → 遍历 `motor->sdo_async_list` → 匹配 index/subindex → 调用回调
4. 超时检测：接收线程定期检查过期任务，触发超时回调

**SDO 响应验证：** 每次请求记录 pending index/subindex，响应到达时校验匹配，防止迟到响应污染下一次操作。

### PDO / TPDO

- **TPDO 触发**：SYNC 帧（0x080）触发从站上报 TPDO
- **TPDO 缓存**：接收线程解析 TPDO 帧，更新 `motor->pdo_pos`/`pdo_vel`/`pdo_torque`
- **RPDO 配置**：支持动态配置映射和传输类型
- **传输类型**：支持同步（1-240）和事件触发（254/255）

### CAN 传输层 (can_socket.c)

- **SocketCAN RAW** — 标准 Linux CAN 接口
- **epoll ET 模式** — 边沿触发 + 非阻塞 fd，高效收帧
- **CAN_RAW_FILTER** — 内核级过滤，只递送关心的帧类型（NMT/EMCY/TPDO1-4/SDO 响应/Heartbeat）
- **互斥锁** — 保护 `write()` 调用，`recv_timeout/poll` 路径

### 关键设计决策

| 决策 | 选择 | 理由 |
|------|------|------|
| 每总线 fd/线程数 | 1 socket + 1 接收线程 | 轻量化，多电机不额外消耗资源 |
| SDO 等待机制 | `pthread_cond_timedwait` | 无自旋，CPU 零占用 |
| 错误码 | `0x603F` (CiA 301 标准) | 兼容所有 CANopen 从站 |
| TPDO 读取 | 缓存 + 内存屏障 | 无锁高速读取 |
| refcount | `__atomic_*` 操作 | 多线程安全 |
| 接收线程优先级 | SCHED_FIFO (prio=30) | 降低运动通信抖动 |
| CAN 过滤器 | SFF + mask 0x780 | 仅 8 组过滤器即覆盖全部关注帧 |

## 构建

### 依赖

- Linux 内核 >= 3.6 (SocketCAN)
- GCC >= 4.8 或 Clang
- CMake >= 3.10
- (可选) Doxygen — 生成 API 文档

### 编译

```bash
cd miraculous_SDK
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 一键打包 (推荐)

输出 `include/ lib/ example/ doc/` 标准目录结构：

```bash
./package.sh
# 生成: miraculous_sdk_x86_64_linux_gnu_20260608/
```

或指定平台：

```bash
./package.sh aarch64_linux_gnu
```

### 输出结构

```
miraculous_sdk_<platform>_<date>/
├── include/
│   └── miraculous_sdk.h          # 唯一对外头文件
├── lib/
│   ├── libmiraculous_sdk.a       # 静态库
│   └── libmiraculous_sdk.so      # 动态库
├── example/
│   ├── CMakeLists.txt            # 独立编译配置
│   ├── test_pp_mode.c            # 轮廓位置模式
│   ├── test_pv_mode.c            # 轮廓速度模式
│   ├── test_pt_mode.c            # 轮廓转矩模式
│   ├── test_csp_mode.c           # 周期同步位置
│   ├── test_csv_mode.c           # 周期同步速度
│   ├── test_cst_mode.c           # 周期同步转矩
│   ├── test_mit_mode.c           # MIT 力位混合
│   ├── test_loop_pp_mode.c       # 循环 PP
│   ├── test_raw_can.c            # 原始 CAN 收发
│   ├── test_pdo_config.c         # PDO 使能+读取
│   ├── test_multi_motor.c        # 多电机控制
│   ├── test_read_params.c        # 参数读取
│   ├── test_emcy_callback.c      # EMCY 回调示例
│   └── test_heartbeat_callback.c # Heartbeat 回调示例
└── doc/
    └── html/                     # Doxygen API 文档 (可选)
```

## 快速开始

```c
#include "miraculous_sdk.h"
#include <stdio.h>

int main() {
    /* 1. 打开电机 (含 CAN + CANopen) */
    MiraMotor *motor = miraculous_motor_open("can0", 1000, 1);
    if (!motor) { return -1; }

    /* 2. Bootstrap + 使能 */
    miraculous_motor_bootstrap(motor, 3000);
    miraculous_motor_full_enable(motor);

    /* 3. 运动控制 (PP 模式) */
    miraculous_motor_pp_move(motor, 100000, 50000, 100000, 100000, false, false);
    miraculous_motor_pp_wait_target(motor, 5000);

    /* 4. 清理 (自动释放所有资源) */
    miraculous_motor_shutdown(motor);
    miraculous_motor_close(motor);
    return 0;
}
```

编译运行：

```bash
gcc -o my_app my_app.c -I/path/to/sdk/include -L/path/to/sdk/lib -lmiraculous_sdk -lpthread -lrt
export LD_LIBRARY_PATH=/path/to/sdk/lib:$LD_LIBRARY_PATH
./my_app
```

## 运行示例

```bash
# 确保 CAN 接口已启用
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# 方法一: 使用 SDK 自编译的示例
./build/bin/example_pp_mode can0 1

# 方法二: 使用打包后 SDK 的独立编译
cd <sdk_dir>/example
mkdir build && cd build
cmake .. -DSDK_PATH=<sdk_dir>
make
./example_pp_mode can0 1
```

## 文档

- [API 概览](doc/sdk_api_overview.md) — SDK API 分类说明
- [构建与链接指南](doc/sdk_build_and_link.md) — 构建选项、链接 .so/.a 的方法
- [示例列表](doc/examples_list.md) — 所有示例的用途和运行方法
- `doxygen Doxyfile` — 生成 HTML API 参考文档

## 许可

内部使用
