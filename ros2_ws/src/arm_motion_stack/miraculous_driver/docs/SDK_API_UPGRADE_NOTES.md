# SDK API 升级说明 (2026-06-25)

## 2026-07-02 更新

7 月 2 日 x86-64 SDK 新增并导出：

- `miraculous_motor_sync_send/start/stop()`：官方 SYNC 管理 API。
- `miraculous_motor_get_velocity_ex(..., VEL_SIDE_LOAD, VEL_UNIT_RAD_S)`：直接读取负载侧 rad/s。
- `miraculous_sdk_version()` / `miraculous_sdk_build_time()`：版本信息。

ROS 2 适配状态：

- `MiraculousArm` 已移除 `set_targets_pulse()`、`get_positions_pulse()`、`rad_to_pulse()`、`pulse_to_rad()` 兼容路径。
- ROS/MoveIt 外部接口仍使用关节侧 rad。
- SDK position `_ex` API 已确认为关节输出侧 rad，ROS driver 不再配置或应用减速比。
- 速度状态改用 SDK 的负载侧 rad/s，不再把 `get_velocity()` 当 pulse/s 手工换算。

## 概述

SDK 新增了 `_ex` 后缀函数，支持直接使用弧度/角度单位进行位置控制，无需手动脉冲转换。

## 新增 API

### 1. CSP 设置目标（带单位）
```c
int miraculous_motor_csp_set_target_ex(MiraMotor *motor, float target_pos, PosUnit_t unit);
```
- **参数**: `target_pos` — 目标位置（弧度或角度）
- **参数**: `unit` — `POS_UNIT_RADIAN` 或 `POS_UNIT_DEGREE`
- **优势**: SDK 内部自动转换为脉冲，使用编码器位宽精确计算

### 2. 读取位置（带单位）
```c
int miraculous_motor_get_position_ex(MiraMotor *motor, float *pos, PosUnit_t unit);
```
- **参数**: `pos` — 输出实际位置（弧度或角度）
- **参数**: `unit` — `POS_UNIT_RADIAN` 或 `POS_UNIT_DEGREE`
- **优势**: 直接返回弧度值，无需手动转换

### 3. 编码器位宽配置
```c
int miraculous_motor_set_encoder_bw(MiraMotor *motor, uint8_t bw);
```
- **默认值**: 19 位（524288 counts/rev）
- **用途**: 当编码器硬件位宽与默认值不同时调用

## ROS 2 代码适配

### 修改的文件
- `miraculous_driver/src/miraculous_arm.cpp`

### 关键改动

#### 1. 后台读取线程 (`read_loop`)
**之前:**
```cpp
int32_t p = 0;
miraculous_motor_get_position(motors_[i], &p);  // 脉冲
cached_pos_rad_[i] = pulse_to_rad(p, i);         // 手动转换
```

**之后:**
```cpp
float pos_rad = 0.0f;
miraculous_motor_get_position_ex(motors_[i], &pos_rad, POS_UNIT_RADIAN);  // 直接弧度
cached_pos_rad_[i] = static_cast<double>(pos_rad);
```

#### 2. CSP 下发 (`set_targets_rad`)
**之前:**
```cpp
std::array<int32_t, kArmJoints> pulses{};
for (size_t i = 0; i < kArmJoints; ++i) {
  pulses[i] = rad_to_pulse(clamped[i], i);  // 手动转换
}
miraculous_motor_csp_set_target(motors_[i], pulses[i]);  // 脉冲
```

**之后:**
```cpp
for (size_t i = 0; i < kArmJoints; ++i) {
  miraculous_motor_csp_set_target_ex(motors_[i],
                                      static_cast<float>(clamped[i]),
                                      POS_UNIT_RADIAN);  // 直接弧度
}
```

#### 3. 移除旧 pulse 兼容路径
- ROS wrapper 不再暴露 `set_targets_pulse()` / `get_positions_pulse()`。
- 新代码统一使用 `set_targets_rad()` / `get_positions_rad()`。

## 优势

1. **消除手动转换误差**: SDK 内部使用 `2^bw` 精确计算，避免浮点舍入累积
2. **代码更简洁**: 不需要维护 pulse 位置换算逻辑
3. **可配置精度**: 通过 `set_encoder_bw()` 调整编码器分辨率
4. **与示例一致**: 和 SDK 提供的 `test_csp_ex.c` 保持一致

## 影响范围

- ✅ **构建验证通过**: x86_64 开发机编译成功
- ✅ **API 兼容**: 外部接口不变 (`get_positions_rad()`, `set_targets_rad()`)
- ✅ **轨迹测试节点**: `trajectory_tracking_test_node` 无需修改，自动受益
- ⚠️ **待测试**: 需要在真实 CAN 总线上验证端到端行为

## 迁移建议

对于未来新增的代码：
- **优先使用** `_ex` API (`csp_set_target_ex`, `get_position_ex`)
- **避免使用** 原始脉冲接口 (`csp_set_target`, `get_position`)
- ROS wrapper 中不再保留 pulse 位置接口；如需底层脉冲调试，直接使用 SDK example。

## 参考文档

- [IMPLEMENTATION_DETAILS.md](./IMPLEMENTATION_DETAILS.md) — 完整实现文档
- [TRAJECTORY_TEST_DOCUMENTATION.md](./TRAJECTORY_TEST_DOCUMENTATION.md) — 轨迹测试框架文档
- SDK 示例: `miraculous_sdk_x86_64_linux_gnu_20260702/example/test_csp_ex.c`
