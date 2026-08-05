# MoveIt 真机接入与验收 Runbook

更新日期：2026-08-05

本文是 `J1..J6` 六轴机械臂首次通过 MoveIt 驱动真机的操作入口。
第一阶段只验收关节空间的小步运动、取消和故障停机，不执行位姿目标或笛卡尔
轨迹。当前 `tool0` 仍是零偏移占位坐标系，不能作为真实 TCP 使用。

整体架构、命令/反馈/故障数据流和分层测试依据见：
`docs/MOVEIT_REAL_INTEGRATION_DESIGN.md`。板端/PC 拆分、DDS 与双机验收细节见
`docs/REMOTE_MOVEIT_DEPLOYMENT.md`。

## 1. 已实现的安全边界

- 板端 `real_control_board.launch.py` 和 PC 端
  `moveit_remote_pc.launch.py` 都只接受完整六轴 profile。
- 生产 profile 默认 `calibrated: false`，其中限位和看门狗只是待验收值；未完成
  六轴复核时 launch 会在打开 CAN 或使能电机之前失败。
- launch 后 `ARMSystem` 是 `inactive`，`arm_controller` 也是 `inactive`。
  操作员必须分两步显式激活。
- 激活 CSP 前后都用新鲜编码器反馈播种位置命令，避免首帧跳变。
- 任一目标 RPDO 写失败时，Driver 不再写后续轴，也不发送本周期 SYNC；随后
  quick-stop并锁存故障。
- TPDO 反馈超过 30 ms、单周期命令跳变超过 `0.005 rad`、跟踪误差超过
  `0.05 rad` 达 3 个 fresh 周期、非有限数据、电机故障或 EMCY 都会停机。
- 故障后不提供 ROS reset 服务，也不自动 fault-reset；必须退出整套进程，在
  外部完成驱动器复位后重新启动。
- `arm_controller` 使用闭环状态反馈，不允许 partial goal 或非零末端速度。
- MoveIt 速度/加速度来自同一份真机 profile；初始保守值分别为
  `0.05 rad/s` 和 `0.10 rad/s²`。

这不是机械、电气安全系统的替代品。急停、限位开关、驱动器保护和安全距离仍
必须独立有效。

## 2. 环境和 clean build

不要从 Conda 环境继承 MoveIt/C++ 运行库。新终端执行：

```bash
cd /home/alienware/Documents/PersonalProject
source /opt/ros/humble/setup.bash

cmake -S miraculous_sdk -B miraculous_sdk/build
cmake --build miraculous_sdk/build -j

cd ros2_ws
colcon build --symlink-install \
  --packages-up-to \
    arm_real_config arm_remote_control arm_description arm_moveit_config \
    arm_planning_examples miraculous_driver
source install/setup.bash
```

如果以前在另一个绝对路径构建过工作区，先使用 `--cmake-clean-cache`，或使用
全新的 build/install/log 目录。构建后应确认：

```bash
colcon test --packages-select arm_real_config miraculous_driver
colcon test-result --verbose
```

## 3. 上电前检查

至少两人在场，一人操作急停。机械臂周围清空，负载可靠固定。确认：

```bash
git status --short --branch
git -C ../miraculous_sdk status --short --branch
ip -details link show can1
```

六轴映射固定为：

| ROS joint | CANopen node |
|---|---:|
| J1 | 1 |
| J2 | 2 |
| J3 | 3 |
| J4 | 4 |
| J5 | 5 |
| J6 | 6 |

若实机映射不同，先停止；不要只为了让 launch 通过而改节点顺序。

## 4. 采集并审核关节软限位

先按示教流程在失能状态下缓慢移动每个关节，记录完整六轴反馈：

```bash
ros2 launch miraculous_driver teach.launch.py \
  node_ids:=1,2,3,4,5,6 \
  joint_indices:=0,1,2,3,4,5 \
  output_file:=/tmp/arm_limits_teach.csv
```

另一个终端：

```bash
ros2 service call /teach_record/start std_srvs/srv/Trigger
# 缓慢覆盖每一轴的计划工作范围，不碰机械硬限位。
ros2 service call /teach_record/stop std_srvs/srv/Trigger
```

检查 CSV 中每轴的最小/最大值，并在机械硬限位内保留足够余量。把审核后的弧度值
写入：

```text
src/arm_motion_stack/arm_real_config/config/real_arm_profile.yaml
```

每个关节都必须满足：

```text
URDF lower <= position_min < position_max <= URDF upper
```

六轴数值经过第二人复核后，才把 `calibrated` 改为 `true`。不要把示教采样的
极值直接当作安全限位，也不要用 `0/0` 绕过校验。

`hardware.baudrate: 0` 是受支持的现场配置，表示 SDK 不修改 SocketCAN 接口
当前波特率；它不表示波特率未知或关闭 CAN。使用该值前必须先确认接口已经按
电机侧要求配置：

```bash
ip -details link show can1
```

负数 baudrate 会在接触硬件前被 profile 和 ros2_control 插件共同拒绝。

安装/重新构建后，可离线验证 profile：

```bash
python3 -c \
  "from arm_real_config import load_real_arm_profile; \
p=load_real_arm_profile('install/arm_real_config/share/arm_real_config/config/real_arm_profile.yaml'); \
print(p.node_ids_csv, p.position_min_csv, p.position_max_csv, p.fingerprint)"
```

## 5. 启动，但不使能

先在机械臂板端启动本地控制链：

```bash
ROS_LOG_DIR=/tmp/ros-log-board \
ros2 launch miraculous_driver real_control_board.launch.py
```

再在外部 PC 启动 MoveIt 与 RViz：

```bash
ROS_LOG_DIR=/tmp/ros-log-pc \
ros2 launch arm_moveit_config moveit_remote_pc.launch.py use_rviz:=true
```

两端打印的 profile SHA-256 指纹必须完全相同。PC 心跳会携带该指纹，板端只接受
完全匹配的心跳；此时允许启动的是状态发布、controller manager、MoveGroup、RViz
和失联监督器，不允许电机进入 Operation Enabled。
在板端检查：

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo /arm_joint_states --once
ros2 topic echo /remote_motion_watchdog/state --once
```

预期：

- `ARMSystem` 为 `inactive`；
- `joint_state_broadcaster` 为 `active`；
- `arm_controller` 为 `inactive`；
- `remote_motion_watchdog` 为 `MONITORING`；
- `/arm_joint_states` 包含且只包含 `J1..J6`，值有限并与实机姿态一致。

MoveIt 的 `No 3D sensor plugin(s) defined for octomap updates` 警告在当前无深度
传感器阶段是预期行为，不要添加假的 `sensors_3d` 配置。

## 6. 人工激活

急停人员就位，确认 RViz 当前姿态与实机一致后，严格在板端本地终端按顺序执行：

```bash
ros2 control set_hardware_component_state ARMSystem active
ros2 control switch_controllers --activate arm_controller --strict
```

如果匹配心跳不新鲜，`MiraculousSystem` 会拒绝从 inactive 进入 active；不得绕过该
检查。执行期间 PC 心跳超时会取消当前轨迹并停用 JTC，恢复通信也不会自动续跑。

再检查：

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic hz /arm_joint_states
```

预期硬件和 `arm_controller` 都为 `active`，机械臂保持当前位置，没有可见跳变。
任一步失败都不要重试激活；按第 10 节退出并复位。

## 7. 先验收原始 JointTrajectoryController

先读取一次当前六轴位置：

```bash
ros2 topic echo /arm_joint_states --once
```

复制同一时刻的六个位置，只把 J1 向安全方向修改最多 `0.02 rad`，并给足
`3 s`。下面的 `CURRENT_*` 必须替换为刚读到的实际值：

```bash
ros2 action send_goal --feedback \
  /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: [J1, J2, J3, J4, J5, J6],
    points: [{
      positions: [CURRENT_J1_PLUS_OR_MINUS_0_02, CURRENT_J2, CURRENT_J3, CURRENT_J4, CURRENT_J5, CURRENT_J6],
      time_from_start: {sec: 3, nanosec: 0}
    }]
  }}"
```

不允许省略关节。验收实际反馈连续、方向正确、误差在 JTC 约束内，然后用反向
小步回到安全位置。

取消验收使用更长、仍在软限位内的低速小步目标；运动开始后，在另一终端立即：

```bash
ros2 control switch_controllers --deactivate arm_controller --strict
```

确认 action 终止且位置不再沿原轨迹变化。重新验收前先保持现场安全，再显式
激活控制器。硬件保持 active 时，停控制器到停硬件之间不应做其他操作。

## 8. MoveIt 关节空间 smoke test

安全工具默认只规划当前姿态相对 J1 的 `+0.02 rad`，并保持其余五轴。它会检查
起点、目标幅度、非目标轴漂移、轨迹速度、时长和有限数值；默认不执行：

```bash
ros2 run arm_planning_examples moveit_joint_smoke_test
```

若 J1 接近上限，改为负方向：

```bash
ros2 run arm_planning_examples moveit_joint_smoke_test --ros-args \
  -p delta_rad:=-0.02
```

人工查看计划和实机空间后才允许执行：

```bash
ros2 run arm_planning_examples moveit_joint_smoke_test --ros-args \
  -p delta_rad:=0.02 \
  -p execute:=true
```

这一阶段不要运行 `plan_to_pose_target`、`plan_cartesian_path` 或任何真实笛卡尔
执行；真实 TCP、碰撞余量和姿态规划尚未验收。

## 9. 正常停机

先停轨迹控制器，再停硬件：

```bash
ros2 control switch_controllers --deactivate arm_controller --strict
ros2 control set_hardware_component_state ARMSystem inactive
```

确认硬件 inactive 后，再 `Ctrl-C` 退出 launch，最后按现场电气流程断使能/断电。

## 10. 故障恢复

遇到以下任一情况立即按故障处理：控制接口返回 `ERROR`、EMCY、反馈 stale、
目标写入失败、驱动器 fault、意外运动、quick-stop 结果失败。

1. 按现场急停或安全断能流程确保机械臂不会继续运动。
2. 退出板端 `real_control_board.launch.py` 和 PC 端
   `moveit_remote_pc.launch.py`；不要在原进程里 cleanup/reconfigure。
3. 检查 CAN、驱动器状态字、供电、编码器和机械干涉。
4. 使用驱动器厂家工具或断电流程完成外部 fault reset。
5. 从第 3 节重新检查并启动。

软件故障锁存是刻意设计的：没有 ROS fault-reset 服务，也没有自动恢复路径。
