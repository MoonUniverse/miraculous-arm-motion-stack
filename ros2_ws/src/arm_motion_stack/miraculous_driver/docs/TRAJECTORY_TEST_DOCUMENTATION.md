# 轨迹跟踪测试框架 — 设计文档与使用文档

> 日期: 2026-06-22
> 组件: `miraculous_driver / trajectory_tracking_test_node`
> 状态: 支持单轴及多轴同相同步测试, 上板时仍建议从小幅单轴开始

---

## 1. 功能概述

`trajectory_tracking_test_node` 是一个 ROS 2 测试节点, 用于验证一个或多个电机在 CSP (周期同步位置) 模式下对正弦/余弦轨迹的跟踪精度。

### 核心能力

- 以指定频率 (默认 100Hz) 向选定关节下发同相 sin/cos 位置命令
- 同一周期先写入所有已配置电机目标, 再通过一帧广播 SYNC 同步锁存
- 同步记录每一步的 **命令值** 和 **编码器实际值**, 保证时间对齐
- 为每个测试关节分别计算 RMSE、MAE、最大误差、相关系数、相位滞后
- 输出 CSV 文件, 可用配套 Python 脚本绘图和分析
- 通过 ROS 2 Service 控制启动/停止, 支持自动定时停止

### 为什么放在 ROS 2 而不是 SDK example

| 方面 | SDK C example | ROS 2 节点 (本方案) |
|------|---------------|-------------------|
| 时间同步 | 手动 clock_gettime | ROS 2 steady_clock, 同回调内读写 |
| 单位转换 | 手动计算 | 复用 MiraculousArm wrapper |
| 数据记录 | 手动 fprintf | 结构化 CSV + 精度指标 |
| 绘图 | 需额外 Python | 配套脚本一键绘图 |
| 可视化 | 无 | RViz 实时监控 |
| 可扩展性 | 低 | 参数化, 易扩展波形/关节 |

---

## 2. 架构设计

### 2.1 数据流

```
TrajectoryTrackingTestNode::on_timer() [100Hz 稳态定时器]
    │
    │  1. t = (now - start_time).seconds()
    │  2. waveform = sin(2π × t / period)
    │  3. 对每个 test_joint:
    │       targets[joint] = dc_offset[joint] + amplitude × waveform
    │     其他关节 hold 当前位置
    │  4. arm_->set_targets_rad(targets)
    │     Manual: 各电机 csp_set_target → 一帧广播 SYNC → 更新缓存
    │     Timer:  csp_set_target ×6，后台 read_loop poll timerfd/TPDO
    │  5. arm_->get_positions_rad(actual) → 复制最近一次完成的缓存
    │  6. 分别计算每个测试关节的 command - actual
    │  7. samples_.push_back({t, command, actual, error})
    │  8. ofs_ << CSV row
    │
    ▼
单轴 CSV: timestamp,command_rad,actual_rad,error_rad
多轴 CSV: timestamp,J1_command_rad,J1_actual_rad,J1_error_rad,...
```

Manual CSP 激活期间，`set_targets_rad()` 所在的控制线程独占 SDK I/O，
后台 `read_loop()` 不再 poll 或覆盖位置缓存。CSP 未激活时，反馈仍由
`read_loop()` 主动发 SYNC 获取；Timer CSP 下 `read_loop()` 继续 poll timerfd，
保证 SDK 定时 SYNC 可以发出。Manual CSP 期间后台 `state_poll_rate_hz` 诊断轮询
也暂停；TPDO/EMCY 由每次写周期内的 poll 处理。

### 2.2 DC offset (直流偏移)

测试不以零位为基准, 而以使能后编码器的当前位置为基准:

```
dc_offset[joint] = get_positions_rad()[joint]  // 使能并 settle 后的静止位置
command[joint](t) = dc_offset[joint] + amplitude × waveform(t)
```

这避免了电机启动时突然跳到零位的危险。

### 2.3 其他关节保护

非测试关节不跟随轨迹, 而是保持其当前编码器位置 (hold position):

```cpp
targets = current_position;
for (joint : test_joints) {
    targets[joint] = command[joint];
}
```

### 2.4 EMCY 故障检测

测试过程中持续监控 EMCY 状态。如果检测到故障, 立即停止测试并 disable 电机。

### 2.5 性能指标计算

测试停止后自动计算:

| 指标 | 公式 | 含义 |
|------|------|------|
| RMSE | √(Σ error² / N) | 均方根误差, 综合精度 |
| MAE | Σ |error| / N | 平均绝对误差 |
| Max |error| | max(|error|) | 最大单点偏差 |
| Correlation | corr(cmd, act) | 跟随线性度 (越接近1越好) |
| Phase lag | FFT cross-correlation | 响应延迟 (ms + 相位角°) |

---

## 3. 文件清单

| 文件 | 路径 | 说明 |
|------|------|------|
| 节点实现 | `src/trajectory_tracking_test_node.cpp` | 525 行 |
| 绘图脚本 | `scripts/plot_trajectory.py` | 194 行 |
| Launch | `launch/trajectory_test.launch.py` | 89 行 |
| Config | `config/trajectory_test_params.yaml` | 29 行 |
| CMakeLists | `CMakeLists.txt` | 新增 trajectory_tracking_test_node 目标 |

---

## 4. 参数说明

### 4.1 电机/CAN 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `can_interface` | string | `can0` | SocketCAN 接口名 |
| `baudrate` | int | `1000` | CAN 波特率 (kbps) |
| `node_ids` | string | `1,2,3,4,5,6` | 已安装电机的 CANopen 节点 ID |
| `joint_indices` | string | `0,1,2,3,4,5` | 每个 `node_id` 对应的 ROS 关节槽位, `0=J1 ... 5=J6` |
| `position_min` | string | `0.0,...` | 软件下限 [rad], 可填 1 个、已装关节数量 N 个、或 6 个全量值 |
| `position_max` | string | `0.0,...` | 软件上限 [rad], 可填 1 个、已装关节数量 N 个、或 6 个全量值 |

### 4.2 轨迹参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `amplitude` | double | `0.03` | 正弦/余弦幅值 [rad], 第一阶段保守默认值 |
| `period` | double | `6.0` | 一个完整周期 [s] |
| `frequency` | double | `100.0` | 命令下发频率 [Hz] |
| `waveform` | string | `sin` | 波形类型: `sin` 或 `cos` |
| `test_joint` | int | `0` | 兼容参数: 单个测试关节索引, `test_joints` 为空时生效 |
| `test_joints` | string | `""` | 多关节索引, 例如 `0,1` 表示 J1/J2 同相同步测试 |
| `duration` | double | `3.0` | 自动停止时间 [s] (0=手动) |
| `settle_time` | double | `0.5` | 使能后等待稳定时间 [s] |

### 4.3 输出参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `output_file` | string | `""` | CSV 输出路径 (空=自动时间戳) |
| `joint_states_topic` | string | `/arm_joint_states` | JointState 发布话题 |

---

## 5. 使用指南

### 5.1 构建

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select miraculous_driver \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

### 5.2 启动测试节点

```bash
source install/setup.bash

export ARM_LIMIT_MIN="MIN_J1,MIN_J2,MIN_J3,MIN_J4,MIN_J5,MIN_J6"
export ARM_LIMIT_MAX="MAX_J1,MAX_J2,MAX_J3,MAX_J4,MAX_J5,MAX_J6"

# 第一阶段推荐: J1, sin, 0.03 rad, 3 秒自动停止, 100 Hz
ROS_LOG_DIR=/tmp/ros-log ros2 launch miraculous_driver trajectory_test.launch.py \
  can_interface:=can0 \
  baudrate:=1000 \
  node_ids:=1 \
  joint_indices:=0 \
  position_min:="$ARM_LIMIT_MIN" \
  position_max:="$ARM_LIMIT_MAX" \
  test_joint:=0 \
  amplitude:=0.03 \
  period:=6.0 \
  duration:=3.0

# J1 通过后再逐轴测试, 例如 J3:
ROS_LOG_DIR=/tmp/ros-log ros2 launch miraculous_driver trajectory_test.launch.py \
  can_interface:=can0 \
  baudrate:=1000 \
  node_ids:=1,2,3 \
  joint_indices:=0,1,2 \
  position_min:="$ARM_LIMIT_MIN" \
  position_max:="$ARM_LIMIT_MAX" \
  test_joint:=2 \
  amplitude:=0.03 \
  period:=6.0 \
  duration:=3.0

# J1/J2 同相同步正弦测试:
ROS_LOG_DIR=/tmp/ros-log ros2 launch miraculous_driver trajectory_test.launch.py \
  can_interface:=can1 \
  baudrate:=0 \
  node_ids:=1,2 \
  joint_indices:=0,1 \
  position_min:=-0.5,-0.5 \
  position_max:=0.5,0.5 \
  test_joints:="0,1" \
  waveform:=sin \
  amplitude:=0.1 \
  period:=6.0 \
  frequency:=50.0 \
  duration:=12.0 \
  sync_period_us:=0 \
  output_file:=/tmp/j1_j2_sin_tracking.csv
```

### 5.3 开始测试

```bash
# 在另一个终端:
ros2 service call /trajectory_test/start std_srvs/srv/Trigger
```

节点会:
1. enable_csp() 使能电机
2. 等待 settle_time (0.5s) 让电机稳定
3. 读取当前位置作为 DC offset
4. 开始 100Hz 定时器下发轨迹 + 记录数据

### 5.4 停止测试

```bash
# 手动停止:
ros2 service call /trajectory_test/stop std_srvs/srv/Trigger

# 或设置自动停止:
ros2 launch miraculous_driver trajectory_test.launch.py duration:=3.0 ...
```

停止后节点会:
1. 停止定时器
2. disable() 电机
3. 关闭 CSV 文件
4. 打印精度指标到终端

### 5.5 查看结果

终端输出示例:
```
========================================
  Trajectory Tracking Results
========================================
  Samples:       500
  Duration:      5.000 s
  RMSE:          0.001234 rad  (0.070 deg)
  MAE:           0.000987 rad  (0.056 deg)
  Max |error|:   0.003456 rad  (0.198 deg)
  Correlation:   0.999876
  Est. lag:      12.3 ms  (0.88 deg phase)
  CSV file:       tracking_test_20260622_150000.csv
========================================
Plot with:
  python3 plot_trajectory.py tracking_test_20260622_150000.csv
```

### 5.6 绘图与分析

```bash
# 基础绘图 (自动保存 PNG + 显示):
python3 src/arm_motion_stack/miraculous_driver/scripts/plot_trajectory.py \
  ~/tracking_test_20260622_150000.csv

# 仅保存不显示 (适合无 GUI 环境):
python3 src/arm_motion_stack/miraculous_driver/scripts/plot_trajectory.py \
  ~/tracking_test_20260622_150000.csv --no-show

# 指定保存路径:
python3 src/arm_motion_stack/miraculous_driver/scripts/plot_trajectory.py \
  ~/tracking_test_20260622_150000.csv --save /tmp/result.png --no-show
```

绘图输出三面板图:
- **上图**: 命令位置 vs 实际位置 (蓝色 vs 红色)
- **中图**: 跟踪误差 [mrad], 含 RMSE 上下界
- **下图**: 误差分布直方图, 含 MAE 标记

同时打印 FFT 频域分析:
- 增益 (幅值比): 实际幅值 / 命令幅值
- 相位滞后: 度数 + 毫秒数

### 5.7 RViz 实时监控

节点持续以 50Hz 发布 `/arm_joint_states`, 可在 RViz 中实时观察机械臂运动。

### 5.8 依赖安装

绘图脚本需要 numpy + matplotlib:

```bash
pip3 install numpy matplotlib
# 或使用 conda:
conda install numpy matplotlib
```

---

## 6. 测试场景推荐

### 6.1 单关节基础验证

```bash
# J1 正弦, 0.03rad幅值, 3s自动停止
ros2 launch miraculous_driver trajectory_test.launch.py \
  node_ids:=1 joint_indices:=0 \
  position_min:="$ARM_LIMIT_MIN" position_max:="$ARM_LIMIT_MAX" \
  test_joint:=0 amplitude:=0.03 period:=6.0 waveform:=sin duration:=3.0
```

### 6.2 不同幅值对比 (测试线性度)

先确认 `0.03rad` 小幅值稳定、无 EMCY、无异常跳变，再逐步放大。

```bash
# 第一阶段小幅值
... amplitude:=0.03 period:=6.0 duration:=3.0

# 小幅值进阶
... amplitude:=0.1 period:=6.0 duration:=6.0

# 中幅值必须在确认限位、负载和急停流程后再做
... amplitude:=0.3 period:=6.0 duration:=6.0

# 大幅值测试不属于第一阶段 bring-up
```

### 6.3 不同频率对比 (测试带宽)

```bash
# 慢轨迹
... amplitude:=0.03 period:=6.0

# 中速, 先保持小幅值
... amplitude:=0.03 period:=3.0

# 更快频率放到单轴低风险验证通过后
```

### 6.4 cos 波形测试

```bash
# cos 有非零初值, 测试阶跃响应 + 持续跟踪
... waveform:=cos amplitude:=0.03 period:=6.0 duration:=3.0
```

### 6.5 不同关节对比

```bash
# J1 (基座旋转, 通常惯量最大)
... test_joint:=0

# J3 (肘部, 中等惯量)
... test_joint:=2

# J6 (末端, 惯量最小)
... test_joint:=5
```

---

## 7. CSV 文件格式

单关节测试继续使用原格式:

```csv
timestamp,command_rad,actual_rad,error_rad
0.000,0.000000,0.000123,-0.000123
0.010,0.062791,0.062654,0.000137
0.020,0.125333,0.125198,0.000135
0.030,0.187591,0.187478,0.000113
...
4.990,-0.062791,-0.062905,0.000114
5.000,-0.000000,-0.000098,0.000098
```

字段说明:
- `timestamp`: 测试开始后的时间 [s]
- `command_rad`: 下发的目标位置 [rad] (含 DC offset)
- `actual_rad`: 编码器读取的实际位置 [rad]
- `error_rad`: 命令 − 实际 [rad]

多关节测试按关节展开三列, 例如 J1/J2:

```csv
timestamp,J1_command_rad,J1_actual_rad,J1_error_rad,J2_command_rad,J2_actual_rad,J2_error_rad
0.000,0.100000,0.099800,0.000200,-0.200000,-0.200300,0.000300
```

---

## 8. 扩展方向

当前框架支持多个关节共享同相、同幅值 sin/cos 测试, 可扩展:

1. **独立轨迹参数** — 为各关节分别设置幅值、周期和相位
2. **更多波形** — 三角波、方波、S 曲线 (STEP响应)、Chirp (扫频)
3. **频响分析 (Bode)** — 自动化扫频测试, 绘制幅频/相频特性曲线
4. **STEP 响应测试** — 阶跃信号, 分析上升时间、超调、稳态误差
5. **ROS bag 记录** — 同时录制 ROS topic, 方便回放分析
6. **自动报告生成** — 生成 PDF/HTML 格式的完整测试报告

---

## 9. 快速参考卡

```bash
# === 构建 ===
colcon build --symlink-install --packages-select miraculous_driver \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# === 参数 ===
export ARM_LIMIT_MIN="MIN_J1,MIN_J2,MIN_J3,MIN_J4,MIN_J5,MIN_J6"
export ARM_LIMIT_MAX="MAX_J1,MAX_J2,MAX_J3,MAX_J4,MAX_J5,MAX_J6"

# === 启动 J1 小幅跟随测试 ===
ROS_LOG_DIR=/tmp/ros-log ros2 launch miraculous_driver trajectory_test.launch.py \
  can_interface:=can0 baudrate:=1000 \
  node_ids:=1 joint_indices:=0 \
  position_min:="$ARM_LIMIT_MIN" position_max:="$ARM_LIMIT_MAX" \
  test_joint:=0 amplitude:=0.03 period:=6.0 duration:=3.0

# === 开始 ===
ros2 service call /trajectory_test/start std_srvs/srv/Trigger

# === 停止 ===
ros2 service call /trajectory_test/stop std_srvs/srv/Trigger

# === 绘图 ===
python3 $(ros2 pkg prefix miraculous_driver)/lib/miraculous_driver/plot_trajectory.py \
  ~/tracking_test_*.csv

# === 自动 5s 测试 ===
ros2 launch miraculous_driver trajectory_test.launch.py duration:=3.0
```
