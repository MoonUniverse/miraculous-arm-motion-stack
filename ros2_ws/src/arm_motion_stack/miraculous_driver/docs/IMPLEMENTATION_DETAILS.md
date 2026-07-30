# miraculous_driver 实现文档

> 日期: 2026-06-22 (2026-07-27 更新: 示教 V2 与首点安全回放)
> 状态: 构建通过 (x86_64 开发机), 待真实 CAN 总线测试

---

## 1. 概述

`miraculous_driver` 是在 `miraculous_sdk`（CANopen/CiA402 电机驱动 C SDK）之上封装的 ROS 2 包, 为 ARM 6DOF 机械臂 (J1–J6) 提供两个能力:

1. **MoveIt 集成** — 实现 `miraculous_driver/MiraculousSystem` ros2_control 硬件插件, 通过 `hardware_type:=real` 接入现有 `arm_controller` (JointTrajectoryController) + `joint_state_broadcaster` + MoveIt 规划栈。
2. **示教/回放** — 示教时发送 `shutdown(0x0006)` 并校验
   `Ready to Switch On`，按一次 SYNC 对齐多个关节的新鲜反馈并写入 V2 CSV。
   playback 支持 V2 关节列映射，并先从当前位置平滑插补到 CSV 首点再正式回放。

真机第一阶段上机流程见：

```text
ros2_ws/src/arm_motion_stack/miraculous_driver/docs/REAL_HARDWARE_BRINGUP.md
```

当前 git 版 SDK 适配（2026-07-08 起，见第 10 节）：

- SDK 由电机同事负责构建/交付；driver 只消费 `include/miraculous_sdk.h` +
  `libmiraculous_sdk`。CMake 默认找项目根目录 `miraculous_sdk/`，同时兼容
  `build/lib/`（源码构建产物）与 `lib/`（交付/安装布局）。
- ROS/MoveIt 侧和 SDK `_ex` position API 都使用关节输出侧 rad；2026-07-07 起 driver 将 `encoder_bw`/`reduction_ratio` 下发给 SDK（默认值即 SDK 默认，见 5.1）。
- 速度反馈改用 `miraculous_motor_get_velocity_ex(..., VEL_SIDE_LOAD, VEL_UNIT_RAD_S)`。
- 手动 SYNC 改用 `miraculous_motor_sync_send()`，不再直接发送 raw CAN `0x080`。
- `node_ids` / `joint_indices` 支持只配置已安装关节；`joint_indices` 使用 `0=J1 ... 5=J6`。
- `position_min` / `position_max` 可传入 1 个、已装关节数量 N 个、或 6 轴全量软件限位；某一轴 `max <= min` 时该轴不启用 clamp。
- 轨迹测试默认保持第一阶段保守值：`amplitude:=0.03`、`period:=6.0`、`duration:=3.0`。

### 硬件拓扑

- 6 个 MiraMotor 电机挂在单条 CAN 总线 (`can0`)
- CANopen 节点 ID 1–6 对应 J1–J6
- 波特率 1000 kbps (CiA402 标准)

### SDK 版本

项目根目录只保留一个 SDK：直接从 git 拉取的源码仓 `miraculous_sdk/`（2026-07-08 起，
替换了旧的 `miraculous_sdk_x86_64_linux_gnu_20260702` 预编译快照）。源码构建天然跨架构
（x86-64 / aarch64 均可在目标机上编译），产物在 `miraculous_sdk/build/lib/`。
其他位置的 SDK 通过 `-DMIRACULOUS_SDK_DIR=/abs/path/to/sdk` 显式指定
（支持源码仓布局 `build/lib/` 或安装布局 `lib/`）。

#### SDK API 升级 (2026-06-25)

SDK 新增了 `_ex` 后缀函数，支持直接使用弧度/角度单位，无需手动转换：

**新增 API:**
```c
// CSP 设置目标（带单位）
int miraculous_motor_csp_set_target_ex(MiraMotor *motor, float target_pos, PosUnit_t unit);

// 读取位置（带单位）
int miraculous_motor_get_position_ex(MiraMotor *motor, float *pos, PosUnit_t unit);

// 编码器位宽配置（默认 19 位 = 524288 counts/rev）
int miraculous_motor_set_encoder_bw(MiraMotor *motor, uint8_t bw);
```

**单位枚举:**
```c
typedef enum {
    POS_UNIT_DEGREE = 0,  // 角度 (度)
    POS_UNIT_RADIAN = 1,  // 弧度 (rad)
} PosUnit_t;
```

**ROS 2 代码适配:**
- `MiraculousArm::read_loop()` — 后台线程改用 `get_position_ex(..., POS_UNIT_RADIAN)` 直接读取关节输出侧弧度
- `MiraculousArm::set_targets_rad()` — CSP 下发改用 `csp_set_target_ex(..., POS_UNIT_RADIAN)` 直接发送关节输出侧弧度
- `MiraculousArm::read_loop()` — 速度改用 `get_velocity_ex(..., VEL_SIDE_LOAD, VEL_UNIT_RAD_S)` 读取负载侧 rad/s
- 旧 pulse 位置兼容接口已移除；底层脉冲调试应直接使用 SDK example
- **优势**: SDK 内部处理编码器/减速比等底层换算，ROS driver 只保留关节侧 rad 语义

---

## 2. 包结构

```
ros2_ws/src/arm_motion_stack/miraculous_driver/
├── CMakeLists.txt
├── package.xml
├── miraculous_driver_plugins.xml          # pluginlib 注册: MiraculousSystem
├── docs/
│   └── IMPLEMENTATION_DETAILS.md           # 本文档
├── include/miraculous_driver/
│   ├── miraculous_arm.hpp                  # 6 电机 wrapper 类声明
│   └── miraculous_system.hpp               # ros2_control 插件声明
├── src/
│   ├── miraculous_arm.cpp                  # wrapper 实现 (432 行)
│   ├── miraculous_system.cpp              # 插件实现 (364 行)
│   ├── teach_record_node.cpp               # 示教记录节点 (260 行)
│   └── playback_node.cpp                   # 回放节点 (301 行)
├── config/
│   ├── miraculous_arm_params.yaml          # CAN 与软件限位参数
│   └── real_ros2_controllers.yaml          # 真实硬件 controller_manager 配置
└── launch/
    ├── real_control.launch.py              # 真实硬件 ros2_control 启动
    ├── moveit_real.launch.py               # MoveIt + 真实硬件
    ├── teach.launch.py                     # 示教记录
    └── playback.launch.py                 # 回放
```

### 修改的现有文件

- `arm_description/urdf/ros2_control.xacro` — 新增 `hardware_type == 'real'` 分支 (第 21–30 行)

---

## 3. 架构设计

### 3.1 数据流

```
MoveIt / JointTrajectoryController
    │  position command [rad] × 6
    ▼
MiraculousSystem (ros2_control SystemInterface)
    │  position_commands_[6]
    ▼
MiraculousArm (C++ wrapper)
    │  set_targets_rad() → csp_set_target_ex(..., POS_UNIT_RADIAN) × 6
    │  miraculous_motor_sync_send(motor[0])  ← 手动统一 SYNC
    ▼
CAN 总线 → 6 个电机
```

### 3.2 主动控制后台读取线程

```
MiraculousArm::read_loop() [独立线程, 100Hz]
    │  循环:
    │    1. 未使能时记录各节点 TPDO2 generation 并发送一帧 SYNC
    │    2. condition_variable 等待 SDK 接收线程送回完整的新 TPDO2 组
    │    3. get_position_ex(..., POS_UNIT_RADIAN) × 6  → joint/load-side rad
    │    4. get_velocity_ex(..., VEL_SIDE_LOAD, VEL_UNIT_RAD_S) × 6
    │    5. 按配置低频 get_state(motor[i]) × 6 → Cia402State_t
    │    6. mutex lock → 缓存更新 (joint-side rad + load-side rad/s)
    │    7. sleep_until(next_period)
    ▼
read() 调用方 (ros2_control / playback)
    → mutex lock → 拷贝缓存 (非阻塞)
```

`init_passive()` 不启动该线程。示教节点的 wall timer 直接调用
`read_passive_feedback()`：记录各节点 TPDO2 generation、发送一帧广播 SYNC，并通过
条件变量等待 SDK 接收线程令所有配置节点 generation 递增后才返回完整样本。超时
不会把旧缓存当作新样本，driver 不再调用已从新版 SDK 移除的主动 poll API。

### 3.3 CSP 多轴同步策略

```cpp
// init() 中，每个进程只执行一次:
for (i = 0..5) {
  miraculous_motor_set_mode(motor[i], CIA_MODE_CSP);
  miraculous_motor_csp_init(motor[i], period, /*manual=*/true);
  if (period != 0) miraculous_motor_sdo_write(motor[i], 0x1006, 0, &period, 4);
}

// set_targets_rad() 中, manual SYNC 模式:
for (i = 0..5) miraculous_motor_csp_set_target_ex(motor[i], target_rad, POS_UNIT_RADIAN);
miraculous_motor_sync_send(motor[0]);  // 统一 SYNC
```

- `sync_period_us == 0` 时使用手动 SYNC，wrapper 在 6 个目标全部写入后调用 SDK 的 `miraculous_motor_sync_send()`
- `sync_period_us != 0` 时 `init()` 逐轴写 0x1006，但仍用 manual 方式执行
  `csp_init()`；共享 SDK timer 只在安全使能成功后由首电机启动
- `enable_csp()` 不再调用 `set_mode()` 或 `csp_init()`，因此连续 start 不会重复配置 CSP
- 状态字 `0x6041` 通过 SDO 读取，默认不在读线程轮询；需要诊断状态机时再设置 `state_poll_rate_hz > 0`

### 3.4 单位语义

ROS command/state、CSV 记录、轨迹测试参数以及 SDK `_ex` position API 统一使用关节输出侧弧度。2026-07-07 起 driver 会把 `encoder_bw` / `reduction_ratio` 硬件参数下发给 SDK（`set_encoder_bw` / `set_reduction_ratio`），默认值即 SDK 默认（19 bit / 100.0），仅换电机型号时才需要改。

### 3.5 生命周期

```
init()           open_motors ×6 → bootstrap ×6 → set_mode(CSP) ×6
                 → csp_init ×6（每进程一次）→ start_read_thread
                 (NMT Operational, CSP 已配置但不使能, 编码器可读)

init_passive()   open_motors ×N → bootstrap ×N → motor_shutdown(0x0006) ×N
                 → wait_state(Ready to Switch On) ×N
                 (不配置 CSP, 不启动 read_loop)

enable_csp()     full_enable ×6
                 → seed 当前位置 (防跳变; 读不到位置则失败并回退 disable)
                 passive_ = false

disable()        motor_disable ×6（保留 CSP/SYNC 配置供下次 start 使用）
                 (Operation Enabled → Switched On)

quick_stop()     motor_quick_stop ×6 (急停减速)

fault_reset()    motor_fault_reset ×6 → fault_detected_ = false

shutdown()       stop_read_thread → 注销 TPDO/EMCY 回调 → (timer 模式: sync_stop ×1)
                 → active: motor_shutdown ×N / passive: disable_voltage ×N
                 → motor_close ×N
```

### 3.6 EMCY 检测

```cpp
// emcy_trampoline()（经 miraculous_motor_set_emcy_callback 注册，按总线一次）:
//   SDK 分发器已解析好 node_id / error_code / error_reg
//   过滤非本臂节点
//   设 fault_detected_ = true
//   调用用户注册的 EmcyCallback
```

注意：**不要**用 `miraculous_can_set_recv_callback` 做 EMCY 监听——SDK 的 CANopen
主站自己占用该槽位做全部收包分发（TPDO 缓存/心跳/EMCY），覆盖它会导致位置反馈
永久失效（详见第 10 节根因分析）。

### 3.7 ros2_control 插件生命周期

| 回调 | 操作 |
|------|------|
| `on_init` | 校验 joint 接口 (position command, position+velocity state), 读取 initial_value |
| `on_configure` | 解析硬件参数 → 构建 ArmConfig → arm_->init() → 注册 EMCY 回调 → 等待缓存 → seed 位置 (读不到编码器则 ERROR) |
| `on_activate` | arm_->enable_csp() → 用编码器实时位置重新 seed states/commands (失败则 ERROR 并 disable) → active_=true |
| `on_deactivate` | active_=false → arm_->disable() |
| `on_cleanup` | arm_->shutdown() → arm_.reset() |
| `read` | 拷贝缓存 → position_states_/velocity_states_, 检测 fault |
| `write` | set_targets_rad(position_commands_) |

---

## 4. 关键 API 参考

### 4.1 MiraculousArm 类

```cpp
namespace miraculous_driver;

constexpr size_t kArmJoints = 6;

struct JointConfig {
    std::string name;
    size_t joint_index;
    uint8_t node_id;
    double position_min;
    double position_max;
};

struct ArmConfig {
    std::string can_interface = "can0";
    CiaBaudrate_t baudrate = CIA_BAUDRATE_1000;
    std::vector<JointConfig> joints;  // 已安装关节, 1..6 个
    uint32_t sync_period_us = 0;     // 0 = 手动 SYNC; 非 0 = 共享 SDK 定时器
    uint8_t encoder_bw = 19;         // 编码器位宽 (2^bw counts/rev), SDK 默认
    double reduction_ratio = 100.0;  // 减速比 (负载侧速度换算), SDK 默认
    double read_rate_hz = 100.0;
    double state_poll_rate_hz = 0.0; // 0 = 不轮询 0x6041 状态字
};

class MiraculousArm {
    // 生命周期
    bool init(const ArmConfig& config);
    bool init_passive(const ArmConfig& config);
    void shutdown();

    // PDS 状态机
    bool enable_csp();
    bool enable();
    void disable();
    void quick_stop();
    bool fault_reset();

    // 读取 (线程安全, 非阻塞)
    bool get_positions_rad(std::array<double, 6>& pos) const;
    bool get_velocities_rad(std::array<double, 6>& vel) const;
    bool get_states(std::array<Cia402State_t, 6>& states) const;
    bool has_fault() const;

    // CSP 写入
    bool set_targets_rad(const std::array<double, 6>& targets);
    void send_sync();

    // 安全
    bool check_limits(std::array<double, 6>& targets) const;
    void set_emcy_callback(EmcyCallback callback);
};
```

### 4.2 SDK 函数使用清单

| SDK 函数 | 调用位置 | 说明 |
|----------|---------|------|
| `miraculous_motor_open` | `open_motors()` | 打开 6 个电机句柄 |
| `miraculous_motor_set_encoder_bw` | `open_motors()` | 下发编码器位宽 (默认 19) |
| `miraculous_motor_set_reduction_ratio` | `open_motors()` | 下发减速比 (默认 100.0) |
| `miraculous_motor_bootstrap` | `init()` | NMT Reset → Operational |
| `miraculous_motor_full_enable` | `enable_csp()` / `enable()` | PDS → Operation Enabled |
| `miraculous_motor_set_mode` | `init()` | 每进程一次，设为 CIA_MODE_CSP |
| `miraculous_motor_csp_init` | `init()` | 每进程每关节一次，统一按 manual 初始化 CSP |
| `miraculous_motor_sdo_write` | `init()` | timer 模式逐轴写 0x1006，避免多轴 `csp_init` 重复引用共享 timer |
| `miraculous_motor_csp_set_target_ex` | `set_targets_rad()` | 写入目标位置 (关节输出侧 rad) |
| `miraculous_motor_get_position_ex` | `read_loop()` | 读取实际位置 (关节输出侧 rad) |
| `miraculous_motor_get_velocity_ex` | `read_loop()` | 读取实际速度 (负载侧 rad/s) |
| `miraculous_motor_get_state` | `read_loop()` | 读取 PDS 状态 |
| `miraculous_motor_set_tpdo_callback` | `open_motors()` / `shutdown()` | 注册/注销 TPDO generation 回调；收包由 SDK 后台线程负责 |
| `miraculous_motor_sync_send` | `send_sync()` / `read_loop()`(manual 且 CSP 未激活时) / `refresh_feedback_locked()` | 发送手动 SYNC（TPDO 为 SYNC 触发） |
| `miraculous_motor_sync_stop` | `shutdown()` | 进程退出时停止共享 SDK 定时 SYNC |
| `miraculous_motor_set_emcy_callback` | `open_motors()` / `shutdown()` | 注册/注销 EMCY 回调（按总线一次） |
| `miraculous_motor_disable` | `disable()` | PDS → Switched On |
| `miraculous_motor_quick_stop` | `quick_stop()` | 急停 |
| `miraculous_motor_fault_reset` | `fault_reset()` | 故障复位 |
| `miraculous_motor_shutdown` | `shutdown()` | 关机 |
| `miraculous_motor_close` | `shutdown()` | 关闭句柄 |

---

## 5. 配置参数

### 5.1 xacro 硬件参数 (ros2_control.xacro `hardware_type==real`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | `can0` | SocketCAN 接口名 |
| `baudrate` | `1000` | CAN 波特率 (kbps) |
| `node_ids` | `1,2,3,4,5,6` | 已安装电机的 CANopen 节点 ID |
| `joint_indices` | `0,1,2,3,4,5` | 每个 `node_id` 对应的 ROS 关节槽位, `0=J1 ... 5=J6` |
| `position_min` | `0.0,...` | 软件下限 [rad], 可填 1 个、已装关节数量 N 个、或 6 个全量值 |
| `position_max` | `0.0,...` | 软件上限 [rad], 可填 1 个、已装关节数量 N 个、或 6 个全量值 |
| `sync_period_us` | `0` | 0=手动 SYNC, 非 0=共享 SDK 定时器周期 [us] |
| `encoder_bw` | `19` | 编码器位宽 (2^bw counts/rev), 范围 1..31, 默认即 SDK 默认 |
| `reduction_ratio` | `100.0` | 减速比 (>0), 默认即 SDK 默认, 仅换电机型号时修改 |
| `read_rate_hz` | `100.0` | 后台读取线程频率 |
| `state_poll_rate_hz` | `0.0` | 状态字 SDO 轮询频率，0=关闭；轨迹跟踪默认关闭 |

### 5.2 teach_record_node 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | `can0` | CAN 接口 |
| `baudrate` | `1000` | 波特率 |
| `node_ids` | `1,2,3,4,5,6` | 已安装电机节点 ID |
| `joint_indices` | `0,1,2,3,4,5` | 每个 `node_id` 对应的 ROS 关节槽位 |
| `joint_states_topic` | `/arm_joint_states` | 发布话题 |
| `record_rate` | `50.0` | 录制频率 (Hz) |
| `output_file` | `""` (自动时间戳) | CSV 输出路径 |
| `auto_record` | `false` | 启动即开始录制 |
| `feedback_timeout_ms` | `5` | 每次 SYNC 等待全部配置节点新 TPDO2 的超时 |
| `max_consecutive_misses` | `10` | 连续超时达到该值时终止当前录制 |
| `overwrite_existing` | `false` | 是否允许覆盖显式 `output_file` |

### 5.3 playback_node 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | `can0` | CAN 接口 |
| `baudrate` | `1000` | 波特率 |
| `node_ids` | `1,2,3,4,5,6` | 已安装电机节点 ID |
| `joint_indices` | `0,1,2,3,4,5` | 每个 `node_id` 对应的 ROS 关节槽位 |
| `position_min` | `0.0,...` | 软件下限 [rad], 可填 1 个、已装关节数量 N 个、或 6 个全量值 |
| `position_max` | `0.0,...` | 软件上限 [rad], 可填 1 个、已装关节数量 N 个、或 6 个全量值 |
| `joint_states_topic` | `/arm_joint_states` | 发布话题 |
| `input_file` | `""` | 回放 CSV 路径 |
| `speed_scale` | `1.0` | 回放速度倍率 |
| `loop` | `false` | 循环回放 |
| `approach_velocity_rad_s` | `0.1` | 当前位置到 CSV 首点过渡的最大关节速度 |
| `approach_rate_hz` | `50.0` | 首点过渡命令频率 |
| `approach_min_duration_s` | `0.5` | 首点过渡最短时间 |
| `start_tolerance_rad` | `0.005` | 与首点差值低于该值时跳过长过渡 |
| `feedback_timeout_ms` | `10` | 手动 CSP SYNC 等待完整 TPDO2 反馈的最大时间 |

---

## 6. 构建与安装

### 6.1 构建

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select miraculous_driver \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

### 6.2 安装布局

```
install/miraculous_driver/
├── lib/
│   ├── libmiraculous_driver.so              # 核心 wrapper + 插件
│   ├── libmiraculous_sdk.so                 # 从 miraculous_sdk/build/lib/ 拷入
│   └── miraculous_driver/
│       ├── teach_record_node                 # 示教记录可执行
│       └── playback_node                     # 回放可执行
└── share/miraculous_driver/
    ├── miraculous_driver_plugins.xml        # pluginlib 插件描述
    ├── config/
    │   ├── miraculous_arm_params.yaml
    │   └── real_ros2_controllers.yaml
    └── launch/
        ├── real_control.launch.py
        ├── moveit_real.launch.py
        ├── teach.launch.py
        └── playback.launch.py
```

### 6.3 验证

```bash
source install/setup.bash

# 包发现
ros2 pkg list | grep miraculous

# 可执行文件
ros2 pkg executables miraculous_driver
# 输出:
#   miraculous_driver playback_node
#   miraculous_driver teach_record_node

# 插件 XML
cat install/miraculous_driver/share/miraculous_driver/miraculous_driver_plugins.xml
```

### 6.4 CMakeLists 关键设计

- **SDK 路径选择**: 默认使用项目根目录源码仓 `miraculous_sdk/`（需先 cmake 构建，产物在 `build/lib/`）; 其他 SDK 通过 `-DMIRACULOUS_SDK_DIR=/abs/path/to/sdk` 指定
- **库 vs 可执行文件安装路径**: 库 → `lib/` (pluginlib 查找), 可执行文件 → `lib/miraculous_driver/` (ros2 run 约定)
- **RPATH**: `INSTALL_RPATH "$ORIGIN"` — 运行时在 .so 同级目录查找 `libmiraculous_sdk.so`
- **SDK .so 安装**: `install(FILES ...)` 将预编译 SDK 库安装到 `lib/`
- **可执行文件 SDK include**: 需单独 `target_include_directories` 添加 SDK 头文件路径

---

## 7. 使用方式

### 7.1 MoveIt + 真实硬件

完整安全流程、profile 标定、人工使能与故障恢复见：

```text
../../docs/MOVEIT_REAL_BRINGUP.md
```

```bash
source install/setup.bash
ros2 launch miraculous_driver moveit_real.launch.py
```

启动内容:
- robot_state_publisher (URDF, `hardware_type:=real`)
- ros2_control_node (MiraculousSystem 插件)
- joint_state_broadcaster spawner
- arm_controller spawner（仅加载为 inactive，必须人工激活）
- MoveGroup (MoveIt 规划)
- RViz (可选, `use_rviz:=false` 关闭)

默认 `real_arm_profile.yaml` 为 `calibrated: false`，因此未填入并审核六轴
position limits 前，启动会在接触硬件前明确失败。

### 7.2 仅 ros2_control (无 MoveIt)

```bash
ros2 launch miraculous_driver real_control.launch.py
```

### 7.3 示教记录

```bash
# 启动 (shutdown 0x0006, Ready to Switch On, 可自由拖动)
ros2 launch miraculous_driver teach.launch.py \
  node_ids:=1,3 \
  joint_indices:=0,2 \
  output_file:=/tmp/teach_j1_j3_v2.csv

# 开始录制
ros2 service call /teach_record/start std_srvs/srv/Trigger

# 停止录制 (生成 CSV)
ros2 service call /teach_record/stop std_srvs/srv/Trigger
```

CSV 格式:
```csv
timestamp,sample_index,J1,J3
0.000000000,0,0.100000000,0.050000000
0.040000000,2,0.120000000,0.060000000
```

`timestamp` 以第一行对应的 SYNC 时刻为零点并使用 steady clock；
`sample_index` 是本轮录制的采样尝试序号，示例中缺少 `1` 表示该周期未收齐全部节点。
每行只包含本次配置的关节，并且只有全部关节都返回新 TPDO2 才写入。当前
`playback_node` 会解析 V2 header；CSV 中出现未配置关节、非有限位置、非递增时间或
越过软件限位时会拒绝回放。

### 7.4 回放

```bash
# 启动
ros2 launch miraculous_driver playback.launch.py \
  node_ids:=1,3 \
  joint_indices:=0,2 \
  position_min:=-0.5,-0.5 \
  position_max:=0.5,0.5 \
  input_file:=/tmp/teach_j1_j3_v2.csv \
  approach_velocity_rad_s:=0.1 \
  approach_rate_hz:=50.0 \
  feedback_timeout_ms:=10

# 开始回放
ros2 service call /playback/play std_srvs/srv/Trigger

# 停止回放
ros2 service call /playback/stop std_srvs/srv/Trigger
```

`/playback/play` 首先确认当前位置与全部 CSV 点都在软件限位内，然后
`enable_csp()` seed 当前姿态。当前位置到首点使用五次 minimum-jerk 曲线，过渡时间
满足 `T >= 1.875 * max_joint_delta / approach_velocity_rad_s`；过渡完成后重新设置
steady-clock 零点并按 CSV timestamp 回放。V2 未记录但已配置的关节保持当前位置。

---

## 8. 构建过程中遇到的问题与修复

### 8.1 SDK 架构不匹配 (ARM aarch64 vs x86_64)

**现象**: `ld: libmiraculous_sdk.so: error adding symbols: file in wrong format`

**原因**: 原始 SDK 库是 ARM aarch64 架构, 开发机是 x86_64

**修复**: 使用当前 x86_64 SDK (`miraculous_sdk_x86_64_linux_gnu_20260702/`); ARM/aarch64 或其他 SDK 通过 `-DMIRACULOUS_SDK_DIR=/abs/path/to/sdk` 显式指定

### 8.2 csp_init API 变更

**现象**: `too few arguments to function 'miraculous_motor_csp_init'`

**原因**: x86_64 版 SDK 头文件中 `csp_init` 多了 `bool manual` 参数

**修复**: `csp_init(motor, sync_period_us, true)` — `manual=true` 表示 SDK 不自动发 SYNC

### 8.3 缺少 std_srvs 依赖

**现象**: `fatal error: std_srvs/srv/trigger.hpp: No such file or directory`

**修复**: package.xml 添加 `<depend>std_srvs</depend>`, CMakeLists 添加 `find_package(std_srvs REQUIRED)` 和 `ament_target_dependencies(... std_srvs)`

### 8.4 可执行文件缺少 SDK 头文件路径

**现象**: `fatal error: miraculous_sdk.h: No such file or directory` (仅可执行文件, 库正常)

**原因**: SDK include 路径只设为库的 PRIVATE, 可执行文件未继承

**修复**: 为 `teach_record_node` 和 `playback_node` 单独添加 `target_include_directories(... PRIVATE ${MIRACULOUS_SDK_DIR}/include)`

### 8.5 可执行文件命名空间未引入

**现象**: `'MiraculousArm' was not declared in this scope; did you mean 'miraculous_driver::MiraculousArm'?`

**原因**: `teach_record_node.cpp` 和 `playback_node.cpp` 使用了 `MiraculousArm` 等类型但未引入命名空间

**修复**: 在两个文件中添加:
```cpp
using miraculous_driver::ArmConfig;
using miraculous_driver::JointConfig;
using miraculous_driver::MiraculousArm;
using miraculous_driver::kArmJoints;
```

### 8.6 ComponentInfo 无 limits 成员 (ROS 2 Humble)

**现象**: `ComponentInfo has no member named 'limits'`

**原因**: ROS 2 Humble 的 `hardware_interface::ComponentInfo` 没有 `limits` 成员

**修复**: `position_min` / `position_max` 设为 0.0, `check_limits()` 中 `hi > lo` 为 false 时不做 clamp

### 8.7 RCLCPP_ERROR_THROTTLE 需要 Clock

**现象**: `rclcpp::Logger has no member named 'get_clock'`

**修复**: 替换为 `RCLCPP_ERROR(logger, msg)` (无节流)

### 8.8 可执行文件安装路径

**现象**: `ros2 pkg executables` 返回空

**原因**: 可执行文件安装在 `lib/` 而非 `lib/miraculous_driver/`

**修复**: 库和可执行文件分开安装:
```cmake
install(TARGETS ${PROJECT_NAME} ... LIBRARY DESTINATION lib ...)
install(TARGETS teach_record_node playback_node RUNTIME DESTINATION lib/${PROJECT_NAME})
```

---

## 9. 2026-07-07 对照 SDK 示例审查与修复

电机工程师已用 `example/test_csp_ex.c` 在真机调通 SDK。本次将 driver 逐条对照 SDK
官方示例做了代码审查，并修复发现的问题。板上验证步骤见
[BOARD_TEST_CHECKLIST.md](BOARD_TEST_CHECKLIST.md)。

### 9.1 审查结论：与官方示例一致、无需改动的部分

| driver 写法 | 对照依据 |
|-------------|---------|
| `set_mode(CSP) → full_enable → csp_init` 使能顺序 | `test_csp_ex.c:74-83`（bootstrap 提前到 `init()`，等效） |
| 手动 SYNC：6 关节全部 `csp_set_target_ex` 后发一帧统一 SYNC | `test_csp_ex.c:134-143`；SYNC 是广播帧，一帧同沿锁存全部轴 |
| 发一次 SYNC、读所有电机 TPDO 缓存 | `test_sync_read.c` 官方多电机模式；新版 SDK 后台线程负责收包 |
| EMCY 用 `get_can_ctx` + `can_set_recv_callback` 捕获 | `test_emcy_callback.c:82-99`（driver 的 `len<3` 检查比示例 `len<2` 更安全） |
| 弧度 `_ex` API、CMake 链接 `pthread`/`rt`、`$ORIGIN` RPATH | SDK 头文件注释与示例编译方式 |

另确认：`test_csp_ex` 默认走 **timer** 模式（`argv[3]` 显式传 `manual` 才是手动），
即工程师验证的是 SDK 定时器路径；driver 默认 manual 路径对电机固件完全等价
（总线上的 SYNC 帧一模一样），只是节拍改由 controller_manager `update_rate` 主导。

### 9.2 修复项（按严重度）

| # | 严重度 | 问题 | 修复 |
|---|--------|------|------|
| 1 | 安全-高 | seed 失败不致命：读不到编码器时 `position_commands_` 保持 0，首个 `write()` 周期会命令全关节冲向 0 rad；且 configure→activate 之间电机脱使能、臂可能被扳动/下坠，旧 seed 已过期 | `enable_csp()` 读不到当前位置直接失败并回退 `disable()`；`on_configure` 等待缓存超时返回 ERROR；`on_activate` 使能后用编码器实时位置**重新** seed `position_states_`/`position_commands_`，失败拒绝激活 |
| 2 | 高 | timer SYNC 模式下对 6 电机各调 `csp_init(manual=false)`，每个句柄各启一个 SYNC 定时器 → 总线最多 6 倍 SYNC 帧 | 所有电机一律 `csp_init(manual=true)`（只配 PDO），timer 模式仅在第一个电机上 `sync_start` 一次；`disable()`/`shutdown()` 对应只 `sync_stop` 一次 |
| 3 | 中 | 编码器位宽/减速比从未下发，隐式依赖 SDK 默认值（19 bit / 100:1），换电机型号即失效 | `ArmConfig` 新增 `encoder_bw`/`reduction_ratio`，`open_motors()` 调 `set_encoder_bw`/`set_reduction_ratio`；作为可选硬件参数暴露，默认值即 SDK 默认（本机型已确认无需改） |
| 4 | 小 | `miraculous_arm.cpp` 用 `std::max` 但未包含 `<algorithm>`（靠传递包含，板上工具链可能编不过）；`miraculous_system.cpp` 有死代码 `expand_single_value` | 补包含；删死代码 |
| 5 | 小 | `state_poll_rate_hz=0`（默认）时每周期用初值 `NOT_READY` 覆盖 `cached_states_`；shutdown 关电机前未注销 CAN 回调，有回调打进析构中对象的窗口 | 仅在实际轮询状态字的周期写入状态缓存；`shutdown()` 先 `can_set_recv_callback(ctx, NULL, NULL)` 注销、再停 SYNC、最后关电机 |

`miraculous_arm.cpp` 已在 x86 上通过 `g++ -fsyntax-only -Wall -Wextra` 零警告；
`miraculous_system.cpp` 依赖 ROS 2 头文件，需板上编译验证。

### 9.3 板上验证要点（详见 BOARD_TEST_CHECKLIST.md）

- 修复 1：configure 后手扳关节再 activate，激活瞬间不跳变；拔 CAN 后激活必须被拒绝
- 修复 2：`candump can0` 观察 `080` 帧 —— manual 模式频率 = update_rate 且只有一路；
  timer 模式挂满 6 关节仍只有一路
- 修复 3：手转关节 ~90°，`/joint_states` 变化 ≈ ±1.57 rad

---

## 10. 2026-07-08 适配 git 版 SDK 与"位置读不到"根因修复

SDK 从预编译快照 `miraculous_sdk_x86_64_linux_gnu_20260702` 换成直接拉取的源码仓
`miraculous_sdk/`（可读全部实现源码）。对照源码逐一核对了 driver 的每个 SDK 调用，
定位了 2026-07-07 板上 `get_position_ex failed joint J1 node 1 → enable_csp failed`
的根因，并完成适配。

### 10.1 根因（板上位置读取失败，两个叠加原因）

1. **raw CAN 回调把 SDK 收包分发顶掉了**。`co_master.c` 初始化时通过
   `miraculous_can_set_recv_callback(can_ctx, co_global_recv_callback, co)` 注册
   自己的收包分发（TPDO 缓存更新 / 心跳 / EMCY 都走它）；该接口是**覆盖式**的
   （`can_socket.c`: "简单策略: 优先使用全局回调"）。driver 在 `open_motors()` 里再
   注册 EMCY 原始回调，就把 `co_global_recv_callback` 顶掉 → TPDO 缓存永不更新。
   SDO 走同步直读路径不受影响，所以 bootstrap / full_enable / csp_init 全部成功、
   唯独位置读取失败——与板上日志完全吻合。
2. **`get_position` 只走 TPDO 缓存且 TPDO 是 SYNC 触发的**。`motion_state.c` 中
   `get_position` 无 SDO 回退：`pdo_valid` 为假直接返回 `MRC_ERROR_TIMEOUT`；
   PDO 映射为 EDS 出厂预配，TPDO2 (0x280+nid, 0x6064+0x606C) 由 SYNC 触发。
   官方 `test_csp_ex.c` 读初始位置前先 `sync_send()`（注释"发送 SYNC 触发 TPDO"）。
   driver 旧读线程从不发 SYNC → 使能前/示教模式下位置永远读不到。

### 10.2 修改项

| # | 修改 | 文件 |
|---|------|------|
| 1 | EMCY 改用 SDK 专用接口 `miraculous_motor_set_emcy_callback`（按总线注册一次，回调带 node_id），彻底移除 `miraculous_can_set_recv_callback` 及 `can_ctx_`/`can_recv_trampoline` | `miraculous_arm.{hpp,cpp}` |
| 2 | 主动控制对象在 **CSP 未激活**时由读线程发送 SYNC 维持反馈；被动示教不再使用读线程，而由 `read_passive_feedback()` 对每轮 SYNC 后各节点的新 TPDO2 做 generation 校验 | `miraculous_arm.{hpp,cpp}` |
| 3 | 最新 SDK 的非 manual `csp_init` 会增加共享 timer 引用计数；driver 因此统一使用 manual `csp_init`，timer 模式逐轴显式写 0x1006，安全使能完成后仅由首电机 `sync_start` 一次 | `miraculous_arm.cpp` |
| 4 | CMakeLists：默认 SDK 路径改为 `../../../../miraculous_sdk`，`find_library` 兼容 `build/lib/`（源码构建）与 `lib/`（安装布局），找不到时提示先构建 SDK；仅当链接 .so 时才安装它 | `CMakeLists.txt` |

`enable_emcy_monitor` 参数保留（默认 true），现在只控制是否注册专用 EMCY
回调，不再有干扰收包的风险。

### 10.3 板上验证要点

- 编译前先构建 SDK：`cmake -S miraculous_sdk -B miraculous_sdk/build && cmake --build miraculous_sdk/build`
- 被动示教启动：所有节点必须先通过 `shutdown(0x0006)` 验证为
  `Ready to Switch On`，随后
  `/joint_states` 才开始发布；`candump` 应看到 record_rate 频率的单路 SYNC，
  每轮后跟每个配置节点的 TPDO2
- enable 后（manual 模式）：SYNC 频率应变为 write 周期频率且只有一路
- 故障注入：EMCY 应经专用回调上报（ROS 日志 + `has_fault`），位置反馈不受影响

## 11. 2026-07-13 CSP 初始化生命周期修复

板测确认电机不接受每次 start 都重新执行 CSP 初始化。从本次修改起，CSP 配置与
进程生命周期绑定：

- `MiraculousArm::init()` 在 bootstrap 后执行一次 `csp_init(manual=true)`；
  timer 模式额外逐轴写 0x1006。任一关节配置失败则关闭全部句柄并让节点启动失败。
- `enable_csp()` 分阶段设置 CSP mode、采集新鲜位置、预装/锁存 seed，再进入
  Operation Enabled；不使用会自动 fault reset 的 `full_enable()`。
- `disable()` 只退出 Operation Enabled，不再停止或重建 CSP/SYNC 配置。
- timer SYNC 只在安全使能事务成功后启动；manual SYNC 仍由 active 写周期发送，
  inactive 主动对象由读线程维持反馈。被动示教走独立的同步采样路径。

因此同一进程内连续执行 start → stop → start 时，第二次 start 不包含
`csp_init()`。板上应检查 `csp init` 日志只在 launch 阶段出现一次。

## 12. 待办事项

- [ ] 在 `config/miraculous_arm_params.yaml` 和 xacro/launch 中填入保守 `position_min` / `position_max` 软件限位
- [ ] 在 ARM 目标机上用真实 CAN 总线进行端到端测试
- [x] ~~`csp_set_target` 是否自动发 SYNC~~ — 2026-07-07 审查确认不自动发：`test_csp_ex.c:134-143` 在 set_target 后显式调用 `sync_send`
- [x] ~~验证 SDK 风险项：`get_position` 是 SDO 还是 TPDO 缓存~~ — 2026-07-08 读源码确认：
  纯 TPDO 缓存、无 SDO 回退，且 TPDO 为 SYNC 触发；manual 模式未激活时由读线程主动发 SYNC（见第 11 节）
- [ ] 按 [BOARD_TEST_CHECKLIST.md](BOARD_TEST_CHECKLIST.md) 完成 2026-07-07 修复的板上验证
- [ ] 如有装反方向的关节，driver 需增加 per-joint 符号翻转 (当前假设 URDF 零位/方向 = 电机侧)
- [ ] 补充真实关节速度/加速度/effort 限位 (当前 URDF 为 0)
- [ ] 示教模式下重力下垂问题评估 (可选 MIT 柔顺模式)
- [ ] 将 `package.xml` 中 maintainer/license 从 TODO 改为真实值

---

## 13. 文件索引

| 文件 | 行数 | 说明 |
|------|------|------|
| [CMakeLists.txt](../CMakeLists.txt) | 129 | 构建配置, SDK 架构自动选择 |
| [package.xml](../package.xml) | 34 | 包依赖声明 |
| [miraculous_driver_plugins.xml](../miraculous_driver_plugins.xml) | 14 | pluginlib 插件注册 |
| [miraculous_arm.hpp](../include/miraculous_driver/miraculous_arm.hpp) | 178 | wrapper 类声明 |
| [miraculous_arm.cpp](../src/miraculous_arm.cpp) | 590 | wrapper 实现 |
| [miraculous_system.hpp](../include/miraculous_driver/miraculous_system.hpp) | 64 | 插件声明 |
| [miraculous_system.cpp](../src/miraculous_system.cpp) | 435 | 插件实现 |
| [teach_record_node.cpp](../src/teach_record_node.cpp) | 260 | 示教记录节点 |
| [playback_node.cpp](../src/playback_node.cpp) | 301 | 回放节点 |
| [miraculous_arm_params.yaml](../config/miraculous_arm_params.yaml) | 28 | 电机参数 |
| [BOARD_TEST_CHECKLIST.md](BOARD_TEST_CHECKLIST.md) | — | 2026-07-07 修复的板上测试步骤 |
| [real_ros2_controllers.yaml](../config/real_ros2_controllers.yaml) | 39 | controller 配置 |
| [real_control.launch.py](../launch/real_control.launch.py) | 53 | ros2_control 启动 |
| [moveit_real.launch.py](../launch/moveit_real.launch.py) | 55 | MoveIt 启动 |
| [teach.launch.py](../launch/teach.launch.py) | 63 | 示教启动 |
| [playback.launch.py](../launch/playback.launch.py) | 66 | 回放启动 |
| [ros2_control.xacro](../../arm_description/urdf/ros2_control.xacro) | 78 | xacro (新增 real 分支) |
