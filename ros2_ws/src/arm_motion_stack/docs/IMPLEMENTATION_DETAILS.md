# arm_motion_stack Implementation Details

本文档记录当前 `arm_motion_stack` 的实现细节、关键配置、运行链路、已处理的 MoveIt 报错，以及后续扩展路径。它用于后续维护和排查，不替代顶层 `README.md` 的快速启动说明。

## 1. 项目边界

当前项目基于已有机械臂 URDF 搭建 ROS 2 + MoveIt 2 + ros2_control + Isaac Sim 预留仿真栈。

已遵守的边界：

- 没有重新生成一套虚假的 6DOF 机械臂模型。
- 没有随意修改原始 link / joint 名称。
- 原始 URDF 被保存在 `arm_description/urdf/original/ARM.urdf`。
- 工作入口使用 xacro wrapper：`arm_description/urdf/arm.urdf.xacro`。
- ros2_control、MoveIt、demo、bringup 都围绕实际关节 `J1` 到 `J6` 配置。

## 2. Package 结构

当前 ROS 2 workspace 主目录：

```text
/home/alienware/Desktop/PersonalProject/ros2_ws
```

核心包：

```text
arm_motion_stack/
├── arm_description
├── arm_moveit_config
├── arm_control
├── arm_bringup
├── arm_kinematics
├── arm_dynamics
├── arm_planning_examples
└── arm_interfaces
```

各包职责：

- `arm_description`: 原始 URDF、xacro wrapper、mesh、RViz 显示配置。
- `arm_moveit_config`: SRDF、KDL IK、OMPL、MoveIt controller 配置、MoveGroup / MoveIt RViz launch。
- `arm_control`: ros2_control controller 配置，fake hardware 和 Isaac Sim 预留启动。
- `arm_bringup`: 统一启动入口。
- `arm_kinematics`: FK、IK、Cartesian path demo。
- `arm_dynamics`: Pinocchio 动力学接口预留。
- `arm_planning_examples`: MoveGroupInterface 轨迹规划 demo。
- `arm_interfaces`: 后续自定义 msg/srv。

## 3. 已识别的机器人模型

来自已有 URDF 的实际信息：

```text
robot name: ARM
base link: base_link
original terminal link: J6
tool / end effector frame: tool0
planning group: single_arm
main joints: J1, J2, J3, J4, J5, J6
```

原始 URDF 没有明确 TCP/tool link，因此当前通过固定关节添加零偏置 `tool0`：

```text
J6 -- fixed joint J6_to_tool0 --> tool0
```

`tool0` 是临时 TCP，后续需要替换为真实末端法兰或工具坐标系。

## 4. Link / Joint 树结构

当前树结构：

```text
base_link
└── J1 joint, revolute, axis 0 0 1, limit [-3.14, 3.14]
    └── link J1
        └── J2 joint, revolute, axis 0 0 1, limit [-1.57, 1.57]
            └── link J2
                └── J3 joint, revolute, axis 0 0 1, limit [-3.14, 3.14]
                    └── link J3
                        └── J4 joint, revolute, axis 0 0 -1, limit [-1.57, 1.57]
                            └── link J4
                                └── J5 joint, revolute, axis 0 0 1, limit [-3.14, 3.14]
                                    └── link J5
                                        └── J6 joint, revolute, axis 0 0 1, limit [-1.57, 1.57]
                                            └── link J6
                                                └── J6_to_tool0 fixed joint
                                                    └── link tool0
```

URDF 完整性记录：

- visual: 原始 link 均存在。
- collision: 原始 link 均存在。
- inertial: 原始 link 均存在。
- effort limit: 原始 URDF 有字段，但均为 `0`，不能作为真实执行参数。
- velocity limit: 原始 URDF 有字段，但均为 `0`，MoveIt 中使用临时测试值。
- acceleration limit: URDF 中缺失，MoveIt 中使用临时测试值。
- mesh: 保留原始 mesh，不删除；工作 xacro 中规范化为 `package://arm_description/meshes/...`。

## 5. arm_description 实现

入口文件：

```text
arm_description/urdf/arm.urdf.xacro
```

设计原则：

- 原始 `ARM.urdf` 不直接破坏。
- 原始模型转换为维护用 xacro copy。
- ros2_control 独立放在 `arm_description/urdf/ros2_control.xacro`。
- `tool0` frame 通过 wrapper 注入，便于后续替换 TCP。

ros2_control xacro macro：

```xml
<xacro:macro name="arm_ros2_control" params="name hardware_type">
```

当前支持的 `hardware_type`：

- `fake`: 使用 `mock_components/GenericSystem`。
- `isaac_mock`: 仍使用 `mock_components/GenericSystem`，但保留 Isaac bringup 入口。
- `isaac`: 预留 `topic_based_ros2_control/TopicBasedSystem`，topic 为 `/isaac_joint_commands` 和 `/isaac_joint_states`。

ros2_control command / state interface：

```text
command_interfaces: position
state_interfaces: position, velocity
joints: J1, J2, J3, J4, J5, J6
```

## 6. MoveIt 2 配置

MoveIt 包：

```text
arm_moveit_config
```

SRDF：

```text
arm_moveit_config/srdf/arm.srdf
```

关键内容：

- `robot name="ARM"`
- fixed virtual joint: `world -> base_link`
- group: `single_arm`
- group joints: `J1` 到 `J6`
- named pose:
  - `home`: 所有关节为 `0`
  - `ready`: `[0, -0.5, 0.8, 0, 0.6, 0]`
- disabled collision:
  - 相邻 link 之间禁用碰撞，包括 `J6` 和 `tool0`。

IK 配置：

```yaml
single_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3
```

已预留替换方向：

- TRAC-IK: `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`
- pick_ik: `pick_ik/PickIkPlugin`

OMPL 配置：

```yaml
default_planning_pipeline: ompl
planning_plugin: ompl_interface/OMPLPlanner
default planner for single_arm: RRTConnectkConfigDefault
projection_evaluator: joints(J1,J2)
longest_valid_segment_fraction: 0.01
```

MoveIt controller：

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager
controller: arm_controller
type: FollowJointTrajectory
action_ns: follow_joint_trajectory
joints: J1, J2, J3, J4, J5, J6
```

## 7. Joint limits 策略

MoveIt joint limits 来自 URDF 的 position limit，同时补了临时 velocity / acceleration：

```text
J1: [-3.14, 3.14]
J2: [-1.57, 1.57]
J3: [-3.14, 3.14]
J4: [-1.57, 1.57]
J5: [-3.14, 3.14]
J6: [-1.57, 1.57]
```

临时测试值：

```text
max_velocity: 1.0
max_acceleration: 1.0
```

注意：

- 原始 URDF 的 velocity / effort 均为 `0`，不能用于真实执行。
- 当前值只用于 MoveIt、ros2_control 和 demo 联调。
- 后续接真实机械臂或 Isaac 动力学仿真前必须替换为真实参数。

## 8. ros2_control 配置

controller config：

```text
arm_control/config/ros2_controllers.yaml
```

controller_manager：

```yaml
update_rate: 100
```

控制器：

```text
joint_state_broadcaster
arm_controller
```

`arm_controller`：

```yaml
type: joint_trajectory_controller/JointTrajectoryController
joints: J1, J2, J3, J4, J5, J6
command_interfaces: position
state_interfaces: position, velocity
allow_partial_joints_goal: false
open_loop_control: true
```

Isaac 预留配置：

```text
arm_control/config/isaac_ros2_controllers.yaml
```

预留 topic：

```text
/isaac_joint_commands
/isaac_joint_states
```

当前 `hardware_type:=isaac_mock` 用于验证 launch 链路，不依赖真实 Isaac bridge plugin。

## 9. Bringup 启动链路

常用启动：

```bash
ros2 launch arm_bringup display.launch.py
ros2 launch arm_bringup fake_demo.launch.py
ros2 launch arm_bringup moveit_demo.launch.py
ros2 launch arm_bringup isaac_moveit.launch.py
```

`display.launch.py`：

```text
robot_state_publisher
joint_state_publisher_gui 或 joint_state_publisher
rviz2
```

`fake_demo.launch.py`：

```text
robot_state_publisher
ros2_control_node
joint_state_broadcaster spawner
arm_controller spawner
rviz2
```

`moveit_demo.launch.py`：

```text
robot_state_publisher
arm_control fake_control.launch.py
arm_moveit_config move_group.launch.py
arm_moveit_config moveit_rviz.launch.py
```

`isaac_moveit.launch.py`：

```text
robot_state_publisher
arm_control isaac_control.launch.py
arm_moveit_config move_group.launch.py
arm_moveit_config moveit_rviz.launch.py
```

## 10. Joint State Topic Isolation

### 10.1 背景

MoveIt 启动时曾出现大量错误：

```text
Joint 'joint24' not found in model 'ARM'
Joint 'joint25' not found in model 'ARM'
Joint 'right_gripper_l_joint1' not found in model 'ARM'
Joint 'left_gripper_r_joint_finger' not found in model 'ARM'
```

排查结论：

- 这些 joint 名称不在当前 URDF。
- 这些 joint 名称不在 SRDF。
- 这些 joint 名称不在 MoveIt joint_limits。
- 这些 joint 名称不在 ros2_control controller 配置。
- 这些 joint 名称来自全局 `/joint_states` 上其他机器人、旧仿真或 gripper publisher 的消息。

MoveIt 的 current state monitor 默认订阅 `joint_states`。如果系统中有其他节点往全局 `/joint_states` 发布无关 joint，MoveIt 会尝试把这些 joint 作为当前 robot model 的状态处理，于是报 `not found in model 'ARM'`。

### 10.2 修复方式

为所有相关 launch 增加：

```python
joint_states_topic = LaunchConfiguration("joint_states_topic")
DeclareLaunchArgument("joint_states_topic", default_value="/arm_joint_states")
```

并 remap：

```python
remappings=[("joint_states", joint_states_topic)]
```

默认隔离 topic：

```text
/arm_joint_states
```

受影响文件：

```text
arm_description/launch/display.launch.py
arm_control/launch/fake_control.launch.py
arm_control/launch/isaac_control.launch.py
arm_bringup/launch/display.launch.py
arm_bringup/launch/fake_demo.launch.py
arm_bringup/launch/moveit_demo.launch.py
arm_bringup/launch/isaac_moveit.launch.py
arm_moveit_config/launch/demo.launch.py
arm_moveit_config/launch/move_group.launch.py
arm_moveit_config/launch/moveit_rviz.launch.py
```

如果需要回到传统全局 topic：

```bash
ros2 launch arm_bringup moveit_demo.launch.py joint_states_topic:=/joint_states
```

注意：MoveIt 日志可能仍打印内部订阅名 `joint_states`，这是节点内部原始 topic 名；实际 remap 会解析到 `joint_states_topic` 指定的 topic。

## 11. Octomap 日志说明

当前 MoveIt 仍可能输出：

```text
No 3D sensor plugin(s) defined for octomap updates
```

结论：

- 当前项目没有配置深度相机或点云 updater。
- MoveIt 启动 world geometry monitor 时会检查 octomap updater。
- 没有 3D sensor plugin 时会打印该日志。
- 这不影响 FK、IK、OMPL planning、MoveGroup action 或 ros2_control 轨迹执行。

没有采用的方案：

- 不使用假的 `sensors_3d.yaml` 去压日志。
- 原因是 ROS 2 Humble 中 raw MoveIt sensor-list YAML 不能直接作为 node parameter file 传给 `move_group`，否则会触发参数解析错误。

后续正确方案：

- 接入真实深度相机、RGB-D 或点云 topic。
- 配置真实 `occupancy_map_monitor/PointCloudOctomapUpdater` 或对应 updater。
- 明确 frame、topic、range、resolution、filter 策略。

## 12. FK / IK / Planning demo

FK：

```bash
ros2 run arm_kinematics fk_demo --ros-args \
  -p joint_positions:="[0.0, -0.5, 0.8, 0.0, 0.6, 0.0]"
```

IK：

```bash
ros2 run arm_kinematics ik_demo
```

Cartesian path：

```bash
ros2 run arm_kinematics cartesian_path_demo --ros-args -p execute:=false
```

Planning examples：

```bash
ros2 run arm_planning_examples plan_to_joint_target --ros-args -p execute:=false
ros2 run arm_planning_examples plan_to_pose_target --ros-args -p execute:=false
ros2 run arm_planning_examples plan_cartesian_path --ros-args -p execute:=false
ros2 run arm_planning_examples execute_named_pose --ros-args -p pose:=ready -p execute:=false
```

这些 demo 使用：

```text
group name: single_arm
tool frame: tool0
joints: J1, J2, J3, J4, J5, J6
```

## 13. 验证命令

推荐使用系统 Python，避免 Conda 影响 ROS 2 包发现：

```bash
cd /home/alienware/Desktop/PersonalProject/ros2_ws
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

URDF 检查：

```bash
ros2 run xacro xacro src/arm_motion_stack/arm_description/urdf/arm.urdf.xacro > /tmp/arm.urdf
check_urdf /tmp/arm.urdf
```

MoveIt headless 检查：

```bash
ROS_LOG_DIR=/tmp/ros-log ros2 launch arm_bringup moveit_demo.launch.py use_rviz:=false
```

期望看到：

```text
You can start planning now!
```

不应再出现：

```text
Joint 'joint24' not found in model 'ARM'
right_gripper_* not found in model 'ARM'
left_gripper_* not found in model 'ARM'
```

topic 检查：

```bash
ros2 topic list | grep joint_states
```

当前默认应看到：

```text
/arm_joint_states
```

如果系统中还有其他节点，可能同时看到全局 `/joint_states`。这正是隔离 `/arm_joint_states` 的原因。

## 14. 已提交记录

相关 git 提交：

```text
e6bb7ad Initialize ARM motion stack
bbeb984 Validate ARM motion stack
9a7c307 Isolate ARM joint states for MoveIt
```

本文件记录的是当前实现状态，后续修改应同步更新 `CHANGELOG.md` 和本文档。

## 15. 后续扩展路线

### 15.1 真实 TCP

替换当前零偏置 `tool0`：

- 从 CAD 或实测得到 flange -> TCP。
- 修改 xacro wrapper 中 `J6_to_tool0` 的 xyz / rpy。
- 重新验证 FK / IK。

### 15.2 真实 joint limits

替换临时值：

- velocity
- acceleration
- jerk
- effort / torque

更新位置：

```text
arm_moveit_config/config/joint_limits.yaml
arm_description/urdf/original/ARM.model.urdf.xacro
arm_description/urdf/ros2_control.xacro
```

### 15.3 Isaac Sim

当前已预留：

```text
hardware_type:=isaac
/isaac_joint_commands
/isaac_joint_states
```

后续需要：

- 确认 Isaac Sim ROS 2 bridge 的消息类型。
- 替换 `topic_based_ros2_control/TopicBasedSystem` 为实际可用 hardware plugin。
- 对齐 Isaac articulation joint order 和 ROS 2 joint order。
- 验证 `/arm_controller/follow_joint_trajectory` 到 Isaac articulation 的闭环。

### 15.4 Pinocchio

当前 `arm_dynamics::DynamicsModel` 是接口占位：

```cpp
bool loadModelFromUrdf(const std::string& urdf_path);
Eigen::VectorXd computeGravity(const Eigen::VectorXd& q);
Eigen::MatrixXd computeMassMatrix(const Eigen::VectorXd& q);
Eigen::VectorXd computeInverseDynamics(
  const Eigen::VectorXd& q,
  const Eigen::VectorXd& dq,
  const Eigen::VectorXd& ddq);
```

后续接入 Pinocchio 时：

- 从 xacro 生成 URDF。
- 用 Pinocchio buildModel。
- 保持 joint order 为 `J1..J6`。
- 加入 gravity、mass matrix、RNEA 单元测试。

### 15.5 Ruckig

后续可接 MoveIt Ruckig time parameterization：

- 先补齐真实 velocity / acceleration / jerk limit。
- 再启用 Ruckig request adapter。
- 对比轨迹时间、速度连续性和执行平滑性。

### 15.6 双臂扩展

不改当前单臂 joint/link 名称。推荐：

- 保留当前 `arm_description` 作为单臂源。
- 新增 `dual_arm.urdf.xacro`。
- 通过 xacro prefix 实例化 left/right arm。
- SRDF 新增 `left_arm`、`right_arm`、`dual_arm` group。
- controller 分为 `left_arm_controller`、`right_arm_controller` 或统一双臂 controller。

### 15.7 真实硬件

后续新增 hardware interface package：

- 实现 `hardware_interface::SystemInterface`。
- 读取真实关节位置/速度。
- 写入 position 或 trajectory command。
- 保持 controller 名称和 joint 名称稳定：

```text
controller: arm_controller
joints: J1, J2, J3, J4, J5, J6
```

