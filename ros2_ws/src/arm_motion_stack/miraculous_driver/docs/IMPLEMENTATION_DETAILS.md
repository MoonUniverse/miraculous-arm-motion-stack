# miraculous_driver 实现文档

> 日期: 2026-06-22
> 状态: 构建通过 (x86_64 开发机), 待真实 CAN 总线测试

---

## 1. 概述

`miraculous_driver` 是在 `miraculous_sdk`（CANopen/CiA402 电机驱动 C SDK）之上封装的 ROS 2 包, 为 ARM 6DOF 机械臂 (J1–J6) 提供两个能力:

1. **MoveIt 集成** — 实现 `miraculous_driver/MiraculousSystem` ros2_control 硬件插件, 通过 `hardware_type:=real` 接入现有 `arm_controller` (JointTrajectoryController) + `joint_state_broadcaster` + MoveIt 规划栈。
2. **示教/回放** — 电机去使能自由拖动示教, 实时记录 6 关节编码器角度到 CSV, 再按记录速率 CSP 回放。

### 硬件拓扑

- 6 个 MiraMotor 电机挂在单条 CAN 总线 (`can0`)
- CANopen 节点 ID 1–6 对应 J1–J6
- 波特率 1000 kbps (CiA402 标准)

### SDK 版本

项目根目录下有两个 SDK 变体, CMakeLists 根据主机架构自动选择:

| 目录 | 架构 | 用途 |
|------|------|------|
| `miraculous_sdk/` | ARM aarch64 | 目标设备 (实机运行) |
| `miraculous_sdk_x86_64/` | x86-64 | 开发主机 (编译验证) |

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
- `MiraculousArm::read_loop()` — 后台线程改用 `get_position_ex(..., POS_UNIT_RADIAN)` 直接读取弧度
- `MiraculousArm::set_targets_rad()` — CSP 下发改用 `csp_set_target_ex(..., POS_UNIT_RADIAN)` 直接发送弧度
- `rad_to_pulse()` / `pulse_to_rad()` — 保留用于向后兼容 (`set_targets_pulse()`)，但标记为已弃用
- **优势**: SDK 内部使用 `2^bw` 精确计算，消除手动转换误差，代码更简洁

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
│   ├── miraculous_arm_params.yaml          # 电机参数 (pulses_per_radian 占位)
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
    │  set_targets_rad() → csp_set_target_ex(..., POS_UNIT_RADIAN) × 6 (SDK _ex API)
    │  can_send(0x080, null, 0)  ← 统一 SYNC
    ▼
CAN 总线 → 6 个电机
```

### 3.2 后台读取线程

```
MiraculousArm::read_loop() [独立线程, 100Hz]
    │  循环:
    │    1. get_position_ex(motor[i], &pos_rad, POS_UNIT_RADIAN) × 6  → rad (SDK _ex API)
    │    2. get_velocity(motor[i]) × 6  → pulse/s
    │    3. get_state(motor[i])   × 6  → Cia402State_t
    │    4. motor_poll(motor[0], 0)    → 处理 CAN 事件/EMCY 回调
    │    5. mutex lock → 缓存更新 (rad + pulse)
    │    6. sleep_until(next_period)
    ▼
read() 调用方 (ros2_control / teach / playback)
    → mutex lock → 拷贝缓存 (非阻塞)
```

### 3.3 CSP 多轴同步策略

```cpp
// enable_csp() 中:
miraculous_motor_csp_init(motor[i], sync_period_us, true);  // manual=true

// set_targets_pulse() 中:
for (i = 0..5) miraculous_motor_csp_set_target(motor[i], targets[i]);
miraculous_can_send(can_ctx_, 0x080, nullptr, 0);  // 统一 SYNC
```

- `manual=true` 确保 SDK 不在每次 `csp_set_target` 后自动发 SYNC
- wrapper 在 6 个目标全部写入后发一个 SYNC, 保证 6 轴在同一 SYNC 边沿同步应用目标
- SYNC 帧: CAN ID = 0x080, DLC = 0

### 3.4 单位转换

```
rad = pulse / pulses_per_radian
pulse = round(rad × pulses_per_radian)

其中:
pulses_per_radian = counts_per_rev × gear_ratio / (2π)
```

每个关节可配置独立的 `pulses_per_radian` 值。

### 3.5 生命周期

```
init()           open_motors ×6 → bootstrap ×6 → start_read_thread
                 (NMT Operational, 不使能, 编码器可读)

init_passive()   = init() + passive_=true
                 (示教模式, 电机去使能可自由拖动)

enable_csp()     full_enable ×6 → set_mode(CSP) ×6 → csp_init(manual=true) ×6
                 → seed 当前位置 (防跳变)
                 passive_ = false

disable()        motor_disable ×6 (Operation Enabled → Switched On)

quick_stop()     motor_quick_stop ×6 (急停减速)

fault_reset()    motor_fault_reset ×6 → fault_detected_ = false

shutdown()       stop_read_thread → motor_shutdown ×6 → motor_close ×6
```

### 3.6 EMCY 检测

```cpp
// can_recv_trampoline():
//   过滤 CAN ID 0x081–0x0FF (EMCY = 0x080 + node_id)
//   解析: error_code (bytes 0-1), error_reg (byte 2)
//   设 fault_detected_ = true
//   调用用户注册的 EmcyCallback
```

### 3.7 ros2_control 插件生命周期

| 回调 | 操作 |
|------|------|
| `on_init` | 校验 joint 接口 (position command, position+velocity state), 读取 initial_value |
| `on_configure` | 解析硬件参数 → 构建 ArmConfig → arm_->init() → 注册 EMCY 回调 → 等待缓存 → seed 位置 |
| `on_activate` | arm_->enable_csp() → active_=true |
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
    uint8_t node_id;
    double pulses_per_radian;
    double position_min;
    double position_max;
};

struct ArmConfig {
    std::string can_interface = "can0";
    CiaBaudrate_t baudrate = CIA_BAUDRATE_1000;
    std::vector<JointConfig> joints;  // 必须有 6 个
    uint32_t sync_period_us = 0;     // 0 = 手动 SYNC
    double read_rate_hz = 100.0;
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
    bool get_positions_pulse(std::array<int32_t, 6>& pos) const;
    bool get_states(std::array<Cia402State_t, 6>& states) const;
    bool has_fault() const;

    // CSP 写入
    bool set_targets_rad(const std::array<double, 6>& targets);
    bool set_targets_pulse(const std::array<int32_t, 6>& targets);
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
| `miraculous_motor_bootstrap` | `init()` | NMT Reset → Operational |
| `miraculous_motor_full_enable` | `enable_csp()` / `enable()` | PDS → Operation Enabled |
| `miraculous_motor_set_mode` | `enable_csp()` | 设为 CIA_MODE_CSP |
| `miraculous_motor_csp_init` | `enable_csp()` | `manual=true`, 不自动发 SYNC |
| `miraculous_motor_csp_set_target` | `set_targets_pulse()` | 写入目标位置 (脉冲) |
| `miraculous_motor_get_position` | `read_loop()` | 读取实际位置 (脉冲) |
| `miraculous_motor_get_velocity` | `read_loop()` | 读取实际速度 (脉冲/s) |
| `miraculous_motor_get_state` | `read_loop()` | 读取 PDS 状态 |
| `miraculous_motor_poll` | `read_loop()` | 处理 CAN 事件 (EMCY 等) |
| `miraculous_motor_get_can_ctx` | `open_motors()` | 获取共享 CAN 上下文 |
| `miraculous_can_send` | `send_sync()` | 发送 SYNC 帧 (0x080) |
| `miraculous_can_set_recv_callback` | `open_motors()` | 注册 EMCY 接收回调 |
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
| `node_ids` | `1,2,3,4,5,6` | 6 个电机的 CANopen 节点 ID |
| `pulses_per_radian` | `1000.0,...` | 每关节脉冲/弧度 (**需替换为真实值**) |
| `sync_period_us` | `0` | 0=手动 SYNC, 非 0=SDK 定时器 |
| `read_rate_hz` | `100.0` | 后台读取线程频率 |

### 5.2 teach_record_node 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | `can0` | CAN 接口 |
| `baudrate` | `1000` | 波特率 |
| `node_ids` | `1,2,3,4,5,6` | 节点 ID |
| `pulses_per_radian` | — | 脉冲/弧度 (逗号分隔 6 个值) |
| `pulses_per_radian_single` | `0.0` | 单值模式 (6 关节相同) |
| `joint_states_topic` | `/arm_joint_states` | 发布话题 |
| `record_rate` | `50.0` | 录制频率 (Hz) |
| `output_file` | `""` (自动时间戳) | CSV 输出路径 |
| `auto_record` | `false` | 启动即开始录制 |

### 5.3 playback_node 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | `can0` | CAN 接口 |
| `baudrate` | `1000` | 波特率 |
| `node_ids` | `1,2,3,4,5,6` | 节点 ID |
| `pulses_per_radian` | — | 脉冲/弧度 |
| `joint_states_topic` | `/arm_joint_states` | 发布话题 |
| `input_file` | `""` | 回放 CSV 路径 |
| `speed_scale` | `1.0` | 回放速度倍率 |
| `loop` | `false` | 循环回放 |

---

## 6. 构建与安装

### 6.1 构建

```bash
cd /home/alienware/Desktop/PersonalProject/ros2_ws
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
│   ├── libmiraculous_sdk.so -> .../miraculous_sdk_x86_64/lib/...
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

- **SDK 架构自动选择**: 根据 `CMAKE_SYSTEM_PROCESSOR` 选择 `miraculous_sdk_x86_64` 或 `miraculous_sdk`
- **库 vs 可执行文件安装路径**: 库 → `lib/` (pluginlib 查找), 可执行文件 → `lib/miraculous_driver/` (ros2 run 约定)
- **RPATH**: `INSTALL_RPATH "$ORIGIN"` — 运行时在 .so 同级目录查找 `libmiraculous_sdk.so`
- **SDK .so 安装**: `install(FILES ...)` 将预编译 SDK 库安装到 `lib/`
- **可执行文件 SDK include**: 需单独 `target_include_directories` 添加 SDK 头文件路径

---

## 7. 使用方式

### 7.1 MoveIt + 真实硬件

```bash
source install/setup.bash
ros2 launch miraculous_driver moveit_real.launch.py
```

启动内容:
- robot_state_publisher (URDF, `hardware_type:=real`)
- ros2_control_node (MiraculousSystem 插件)
- joint_state_broadcaster spawner
- arm_controller spawner
- MoveGroup (MoveIt 规划)
- RViz (可选, `use_rviz:=false` 关闭)

### 7.2 仅 ros2_control (无 MoveIt)

```bash
ros2 launch miraculous_driver real_control.launch.py
```

### 7.3 示教记录

```bash
# 启动 (电机去使能, 可自由拖动)
ros2 launch miraculous_driver teach.launch.py \
  pulses_per_radian:="1000.0,1000.0,1000.0,1000.0,1000.0,1000.0"

# 开始录制
ros2 service call /teach_record/start std_srvs/srv/Trigger

# 停止录制 (生成 CSV)
ros2 service call /teach_record/stop std_srvs/srv/Trigger
```

CSV 格式:
```csv
timestamp,J1,J2,J3,J4,J5,J6
0.000000000,0.100000000,0.050000000,...
0.020000000,0.120000000,0.060000000,...
```

### 7.4 回放

```bash
# 启动
ros2 launch miraculous_driver playback.launch.py \
  input_file:="teach_20260622_140000.csv" \
  pulses_per_radian:="1000.0,1000.0,1000.0,1000.0,1000.0,1000.0"

# 开始回放
ros2 service call /playback/play std_srvs/srv/Trigger

# 停止回放
ros2 service call /playback/stop std_srvs/srv/Trigger
```

---

## 8. 构建过程中遇到的问题与修复

### 8.1 SDK 架构不匹配 (ARM aarch64 vs x86_64)

**现象**: `ld: libmiraculous_sdk.so: error adding symbols: file in wrong format`

**原因**: 原始 SDK 库是 ARM aarch64 架构, 开发机是 x86_64

**修复**: 用户提供 x86_64 SDK (`miraculous_sdk_x86_64/`), CMakeLists 根据 `CMAKE_SYSTEM_PROCESSOR` 自动选择

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

## 9. 待办事项

- [ ] 在 `config/miraculous_arm_params.yaml` 和 xacro 中填入真实 `pulses_per_radian` 值
- [ ] 在 ARM 目标机上用真实 CAN 总线进行端到端测试
- [ ] 验证 SDK 风险项 (见计划文件):
  - `csp_set_target` 是否自动发 SYNC (当前假设不自动发)
  - `get_position` 延迟 (SDO vs TPDO 缓存)
  - 未 `csp_init` 时 `get_position` 是否可用
- [ ] 补充真实关节速度/加速度/effort 限位 (当前 URDF 为 0)
- [ ] 示教模式下重力下垂问题评估 (可选 MIT 柔顺模式)
- [ ] 将 `package.xml` 中 maintainer/license 从 TODO 改为真实值

---

## 10. 文件索引

| 文件 | 行数 | 说明 |
|------|------|------|
| [CMakeLists.txt](../CMakeLists.txt) | 123 | 构建配置, SDK 架构自动选择 |
| [package.xml](../package.xml) | 34 | 包依赖声明 |
| [miraculous_driver_plugins.xml](../miraculous_driver_plugins.xml) | 14 | pluginlib 插件注册 |
| [miraculous_arm.hpp](../include/miraculous_driver/miraculous_arm.hpp) | 177 | wrapper 类声明 |
| [miraculous_arm.cpp](../src/miraculous_arm.cpp) | 432 | wrapper 实现 |
| [miraculous_system.hpp](../include/miraculous_driver/miraculous_system.hpp) | 64 | 插件声明 |
| [miraculous_system.cpp](../src/miraculous_system.cpp) | 364 | 插件实现 |
| [teach_record_node.cpp](../src/teach_record_node.cpp) | 260 | 示教记录节点 |
| [playback_node.cpp](../src/playback_node.cpp) | 301 | 回放节点 |
| [miraculous_arm_params.yaml](../config/miraculous_arm_params.yaml) | 19 | 电机参数 |
| [real_ros2_controllers.yaml](../config/real_ros2_controllers.yaml) | 39 | controller 配置 |
| [real_control.launch.py](../launch/real_control.launch.py) | 53 | ros2_control 启动 |
| [moveit_real.launch.py](../launch/moveit_real.launch.py) | 55 | MoveIt 启动 |
| [teach.launch.py](../launch/teach.launch.py) | 63 | 示教启动 |
| [playback.launch.py](../launch/playback.launch.py) | 66 | 回放启动 |
| [ros2_control.xacro](../../arm_description/urdf/ros2_control.xacro) | 78 | xacro (新增 real 分支) |
