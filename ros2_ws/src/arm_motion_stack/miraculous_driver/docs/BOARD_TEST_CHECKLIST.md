# Driver/SDK 安全修复板上验收清单

> 更新：2026-08-05
> 当前状态：代码审查与离线静态检查阶段；未完成本清单前，不放行 JTC/MoveIt 真机运动。

本清单只覆盖 Driver/SDK 安全修复的板上证据。完整环境准备、软限位采集、启动、
JTC 与 MoveIt 操作步骤以仓库根文档
[`MOVEIT_REAL_BRINGUP.md`](../../docs/MOVEIT_REAL_BRINGUP.md) 为唯一操作基线。

## 1. 固定基线

```text
can_interface=can1
baudrate=0                 # 保持已配置的 SocketCAN 速率
controller update_rate=50 Hz
sync_period_us=0           # Driver-owned Manual SYNC
manual_feedback_timeout=15 ms
feedback_stale_timeout=30 ms
max_command_step=0.005 rad/cycle   # 暂定，待真机冻结
max_following_error=0.05 rad × 3 fresh cycles  # 暂定，待真机冻结
remote heartbeat=50 ms
remote soft stop timeout=250 ms
remote hard stop timeout=500 ms
```

板端 `real_control_board.launch.py` 的全臂路径会拒绝 timer SYNC、关闭 EMCY、
缺少真实限位或关闭上述主动 watchdog 的配置。Timer SYNC 不在本轮 Driver 真机
验收范围内，由 SDK 团队单独验证。

## 2. 上电前门槛

- 硬件急停可用且有人值守，重力轴有可靠支撑或制动。
- `real_arm_profile.yaml` 中 J1–J6 的 node、joint、方向、零点和限位逐项核对。
- J6 完成与 J1–J5 同级别的被动反馈和低速主动验证。
- `tool0/TCP`、MoveIt 限位和碰撞模型未验收前，只做关节空间小步。
- 同一 CAN 接口上没有第二个 Driver、SDK example 或 CANopen master。
- SocketCAN 已由现场脚本配置；`ip -details link show can1` 状态正确。

## 3. 构建与离线测试

```bash
cd <ros2_ws>
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  arm_real_config arm_remote_control arm_description miraculous_driver \
  arm_moveit_config arm_planning_examples
colcon test --packages-select arm_real_config miraculous_driver
colcon test-result --verbose
```

通过标准：构建成功，Driver 测试零失败，且测试中至少覆盖：

- 运行时单轴 RPDO 写失败不发送该周期 SYNC，也不再写后续轴；
- feedback timeout、velocity read failure 和 following error 触发全臂 quick-stop；
- quick-stop/disable/disable-voltage 的最终 CiA402 状态被逐轴确认；
- 非有限、越限和单周期跳变命令在任何 RPDO 前被拒绝；
- 严格参数解析不会把错误输入回退成默认六轴映射。

## 4. Inactive bringup

保持 `real_arm_profile.yaml` 的 `calibrated: false`，先完成 profile 审核；只有六轴参数
审核签字后才改为 `true`。随后按主文档启动：

```bash
ROS_LOG_DIR=/tmp/ros-log \
ros2 launch miraculous_driver real_control_board.launch.py
```

预期：

- `ARMSystem=inactive`；
- `joint_state_broadcaster=active`；
- `arm_controller=inactive`；
- `/remote_motion_watchdog/state=WAITING_FOR_HEARTBEAT`（PC 尚未启动时）；
- `/arm_joint_states` 只含 J1–J6，数值有限、方向正确、约 50 Hz；
- 不进入 `Operation Enabled`，机械臂无主动运动。

## 5. CSP 使能无跳变

先按主文档在 PC 启动 `moveit_remote_pc.launch.py`，并确认板端
`/remote_motion_watchdog/state=MONITORING`。急停人员就位后，再人工激活硬件和控制器：

```bash
ros2 control set_hardware_component_state ARMSystem active
ros2 control switch_controllers --activate arm_controller --strict
```

通过标准：

- 使能前后所有轴 commanded/actual 起点一致；
- 每个周期是全部 RPDO target 后恰好一帧 `0x080` SYNC；
- SYNC 约 20 ms 一帧，且总线上只有一个发送源；
- 六轴均进入 `Operation Enabled` 后才报告激活成功；
- 任一轴失败时全臂回滚，进程内不可自动重试激活。

## 6. 原始 JTC 小步

严格使用主文档的完整六轴 `FollowJointTrajectory` 命令：只改变一个关节最多
`0.02 rad`，时长至少 `3 s`，其余五轴填入同一时刻实测值。不要使用旧版
`0.3 rad` 示例，也不要省略关节。

通过标准：方向正确、无跳变、反馈持续新鲜、JTC action 成功，commanded-actual 未触发
Driver watchdog。随后验收 controller deactivate/cancel 行为。

## 7. 故障注入

故障注入只在无负载、低电流、机械支撑完善的专用台架执行，并由电机工程师给出
可重复的方法。

| 故障 | 必须观察到的结果 |
|---|---|
| 某一轴 RPDO write 失败 | 后续轴不写；该周期 SYNC 计数不增加；全臂 quick-stop |
| 某一轴 TPDO2 缺失 | 15 ms transaction 超时；JTC/ros2_control 返回错误；全臂停止 |
| velocity 读取失败 | 不把 `0 rad/s` 当有效反馈；反馈快照不更新；全臂停止 |
| 连续跟随超差 | 第 3 个独立 fresh snapshot 触发停止，不重复计同一 snapshot |
| EMCY/drive fault | 故障锁存；退出进程并外部复位，不允许软件自动恢复 |
| 执行长轨迹时拔掉 PC 网线 | 软超时后取消 goal 并停用 JTC；若仍运动，硬超时后全臂 quick-stop；恢复网线不续跑 |
| disable 命令或状态确认失败 | 不报告正常完成；SDK I/O 保持隔离；执行现场安全断能 |

每项保存 ROS 日志、`candump can1`、状态字和时间戳作为验收证据。

## 8. MoveIt smoke test

只有第 3–7 节全部通过后，才按主文档先 `execute:=false` 规划，再人工审查并执行
J1 `±0.02 rad` smoke test。真实 TCP、碰撞余量和笛卡尔路径尚未验收前，不运行
pose target、Cartesian path 或抓取流程。

## 9. 停机与失败处理

正常停机按顺序停 controller、停 hardware、确认 inactive，再退出 launch。退出时
Driver 会逐轴命令并确认 `Switch On Disabled` 后关闭句柄。

任何异常动作优先拍硬件急停。软件故障要求退出整个进程、检查 CAN/驱动器/机械状态、
完成外部 fault reset，再从上电前检查重新开始；不要在原进程内 cleanup/reconfigure。
