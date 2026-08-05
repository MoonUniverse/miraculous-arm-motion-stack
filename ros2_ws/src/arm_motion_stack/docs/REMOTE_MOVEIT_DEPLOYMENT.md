# MoveIt 双机部署与第一阶段验收

更新日期：2026-08-05

本文描述当前已经实现的双机部署边界、PC 失联停机策略，以及必须在机械臂板端和
外部 PC 上完成的验收。当前仍只允许有人值守的关节空间小步测试，不代表已取得
功能安全认证。

## 1. 已实现的部署边界

```text
外部 PC（普通 Ubuntu 22.04）
  move_group + OMPL + IK + 碰撞检查 + 时间参数化 + RViz + 心跳
                         |
                         | DDS：完整 FollowJointTrajectory
                         v
机械臂板端（建议 PREEMPT_RT）
  robot_state_publisher + controller_manager + JointTrajectoryController
  + 失联监督器 + MiraculousSystem 硬看门狗
  + MiraculousArm + SDK + SocketCAN
                         |
                         v
                    CANopen 驱动器
```

外部 PC 不发送 50 Hz 单点位置命令。它把完整的带时间轨迹发送到：

```text
/arm_controller/follow_joint_trajectory
```

板端 JTC 缓存轨迹并在本地 50 Hz `read -> update -> write` 循环中插值。因此已经
开始执行的控制周期不依赖网络逐周期到包。

## 2. 代码入口

| 角色 | launch | 明确不启动 |
|---|---|---|
| 板端 | `miraculous_driver/real_control_board.launch.py` | MoveGroup、RViz |
| PC | `arm_moveit_config/moveit_remote_pc.launch.py` | ros2_control、电机 Driver、CAN |
| 单机兼容 | `miraculous_driver/moveit_real.launch.py` | 无；只用于同机联调 |

真机配置的唯一源码已经移到：

```text
arm_real_config/config/real_arm_profile.yaml
```

`arm_real_config` 不依赖电机 SDK。板端和 PC 都从该包读取并严格校验同一种 profile，
再通过同一个函数生成 real xacro 参数。PC 因此可以独立构建规划栈，不需要编译
`miraculous_driver`。

两个 launch 都会在启动时输出：

```text
BOARD real-arm profile fingerprint sha256:<64 hex>
PC real-arm profile fingerprint sha256:<64 hex>
```

两端指纹必须完全相同。PC 心跳携带该指纹；板端软监督器和 Driver 硬看门狗都只
接受完全匹配的心跳。Driver 在没有新鲜匹配心跳时拒绝激活硬件，因此配置不一致
不再只是日志告警。

默认失联参数来自同一份 profile：

```text
心跳周期                         50 ms
软停止超时                      250 ms
Driver 硬停止超时               500 ms
硬停止速度判断阈值              0.02 rad/s
```

软超时后，板端先取消所有当前 `FollowJointTrajectory` goal，让 JTC 进入保持，再
停用 `arm_controller`。若到硬超时仍出现推进中的位置命令，或反馈速度仍超过阈值，
`MiraculousSystem` 通过既有 `fail_safe_stop()` 执行并验证六轴 CiA402 Quick Stop。

## 3. 两端共同前提

- Ubuntu 22.04、ROS 2 Humble；
- 两端检出同一个已审核 Git commit；
- 两端安装同一种 RMW，第一阶段建议统一使用 Cyclone DDS；
- 相同 `ROS_DOMAIN_ID`；
- `ROS_LOCALHOST_ONLY=0`；
- 有线、隔离的控制网络，不使用 Wi-Fi；
- chrony 或 PTP 保证两端时钟同步；
- profile 已完成六轴审核并显式设置 `calibrated: true`；
- 每台机器人使用独立网络或独立 domain/namespace，避免控制器重名。

示例环境变量必须在两端保持一致：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=31
export ROS_LOCALHOST_ONLY=0
```

`31` 只是示例，现场应分配固定值。如果网络禁用了组播，需要另行提供指定网卡和静态
peer 的 Cyclone DDS XML；不要在不知道两端 IP 和网卡名时复制占位配置上线。

## 4. 外部 PC 构建

PC 不需要 Miraculous SDK，也不构建 `miraculous_driver`：

```bash
cd /path/to/repository/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-up-to \
  arm_moveit_config arm_planning_examples
source install/setup.bash
```

确认构建选择中没有 `miraculous_driver`。如果直接无选择地执行整个工作区构建，
Driver 的 CMake 会按设计检查电机 SDK，PC 会因此失败。

## 5. 机械臂板端构建

板端仍需要本仓库匹配版本的 Miraculous SDK：

```bash
cd /path/to/repository
source /opt/ros/humble/setup.bash

cmake -S miraculous_sdk -B miraculous_sdk/build
cmake --build miraculous_sdk/build -j

cd ros2_ws
colcon build --symlink-install --packages-up-to miraculous_driver
source install/setup.bash
```

执行已有离线测试：

```bash
colcon test --packages-select arm_real_config miraculous_driver
colcon test-result --verbose
```

## 6. 启动顺序

### 6.1 板端：只准备控制链，不使能

```bash
ROS_LOG_DIR=/tmp/ros-log-board \
ros2 launch miraculous_driver real_control_board.launch.py
```

预期状态：

```text
ARMSystem                 inactive
joint_state_broadcaster   active
arm_controller            inactive
remote_motion_watchdog     WAITING_FOR_HEARTBEAT
```

板端本地检查：

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo /arm_joint_states --once
ros2 action info /arm_controller/follow_joint_trajectory
ros2 topic echo /remote_motion_watchdog/state --once
```

### 6.2 PC：启动规划与可视化

```bash
ROS_LOG_DIR=/tmp/ros-log-pc \
ros2 launch arm_moveit_config moveit_remote_pc.launch.py use_rviz:=true
```

核对 PC 与板端日志中的 profile 指纹完全一致，然后在 PC 检查：

```bash
ros2 topic echo /arm_joint_states --once
ros2 topic hz /arm_joint_states
ros2 action info /arm_controller/follow_joint_trajectory
ros2 node list
```

板端还应看到：

```bash
ros2 topic echo /remote_motion_watchdog/state --once
```

状态必须为 `MONITORING`。如果一直是 `WAITING_FOR_HEARTBEAT`，不得激活硬件；优先
检查两端 profile 指纹、topic、DDS 网卡和 `ROS_DOMAIN_ID`。

此时 PC 应能发现板端的 action server 和关节状态，但电机仍不能运动。

### 6.3 仅在板端本地人工激活

急停人员就位并确认 RViz 姿态与实机一致后，在板端终端执行：

```bash
ros2 control set_hardware_component_state ARMSystem active
ros2 control switch_controllers --activate arm_controller --strict
```

激活命令虽然能通过 DDS 从 PC 远程调用，第一阶段禁止这样操作。生命周期安全门由
板端现场人员掌握。

### 6.4 PC 先做 plan-only，再允许小步执行

```bash
ros2 run arm_planning_examples moveit_joint_smoke_test
```

确认计划起点、方向和空间安全后：

```bash
ros2 run arm_planning_examples moveit_joint_smoke_test --ros-args \
  -p delta_rad:=0.02 \
  -p execute:=true
```

如果 J1 接近正向软限位，使用 `-0.02`。真实 TCP 未标定前，不执行 Pose 或
Cartesian 目标。

## 7. DDS 第一阶段验收

至少保存以下结果：

1. 两端 commit 与 profile 指纹；
2. PC 能稳定发现板端 node、topic、service 和 action；
3. `/arm_joint_states` 在 PC 侧持续为 50 Hz，时间戳不倒退；
4. 原始 JTC 六轴完整小步成功；
5. MoveIt plan-only 成功；
6. MoveIt `J1 +/-0.02 rad` 执行成功并返回 action result；
7. 执行中取消能够在板端停止轨迹；
8. 执行一条空间安全、低速且足够长的轨迹时拔掉 PC 网线：约 250 ms 后 goal 被
   取消且 `arm_controller` 变为 inactive，整条旧轨迹不得继续；
9. 恢复网线后控制器仍保持 inactive，旧轨迹不得恢复；
10. 人为阻断软监督器的控制器停用路径时，500 ms 硬看门狗能触发已验证 Quick Stop；
11. 相机、GPU、日志和网络负载同时运行时，无 feedback stale 或控制周期超时。

建议同时采集：

```text
控制周期实际/最大耗时
read/update/write 分段耗时
SYNC -> 六轴 fresh TPDO 耗时
DDS action goal 到达延迟
joint_states 端到端延迟与丢包
```

## 8. 失联后的锁存与恢复

心跳恢复不会自动清除失联故障，也不会重新激活 JTC。确认机械臂已经停止、现场空间
安全且 profile 指纹一致后，只能在板端执行：

```bash
ros2 service call /remote_motion_watchdog/reset_fault std_srvs/srv/Trigger
ros2 control list_controllers
ros2 control switch_controllers --activate arm_controller --strict
```

如果日志中出现 `MiraculousSystem FAULT LATCHED`，说明已经升级到硬 Quick Stop。
此时不能调用上述软复位后继续运动；必须按电机现场流程复位驱动器并重启
`controller_manager`，再从 inactive 状态重新验收。

## 9. 当前仍需完成的安全工作

### 9.1 功能安全边界

当前心跳、DDS 监督器和软件 Quick Stop 属于工程保护层，不是经过安全认证的急停。
外部急停、安全门、驱动器 STO/Quick Stop 和现场制动能力仍必须独立于 PC、DDS 和
普通以太网。还需要用实机测出失联检测时间、停止时间和停止距离，而不能只验收日志。

### 9.2 实时性证明

launch 拆分不等于获得实时性。仍需在板端确认 PREEMPT_RT、`SCHED_FIFO` 实际
生效、线程优先级、CPU/IRQ 亲和性、memory locking 和最坏负载时序。

## 10. 正常停机

在板端按反向顺序执行：

```bash
ros2 control switch_controllers --deactivate arm_controller --strict
ros2 control set_hardware_component_state ARMSystem inactive
```

确认硬件 inactive 后，先退出 PC MoveIt，再退出板端 launch，最后按现场电气流程
断使能/断电。
