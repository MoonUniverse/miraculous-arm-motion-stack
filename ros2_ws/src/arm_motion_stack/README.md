# arm_motion_stack

ROS 2 + MoveIt 2 + ros2_control + Isaac Sim motion stack for the existing `ARM` 6DOF single-arm URDF.

This project keeps the original SolidWorks-exported URDF intact and builds ROS 2 wrappers around it. It does not rename the original links or joints and does not replace the mechanical model with a simplified fake arm.

## Change Record

The repository-level setup and this stack bootstrap are recorded in:

```text
/home/alienware/Documents/PersonalProject/CHANGELOG.md
```

Detailed implementation and troubleshooting notes are recorded in:

```text
ros2_ws/src/arm_motion_stack/docs/IMPLEMENTATION_DETAILS.md
```

Real hardware first-stage CAN/encoder and single-joint CSP bring-up is recorded in:

```text
ros2_ws/src/arm_motion_stack/miraculous_driver/docs/REAL_HARDWARE_BRINGUP.md
```

## Current URDF Source

- Original source in this workspace: `/home/alienware/Documents/PersonalProject/arm_description/urdf/ARM.urdf`
- Preserved copy: `ros2_ws/src/arm_motion_stack/arm_description/urdf/original/ARM.urdf`
- ROS 2 model entry point: `ros2_ws/src/arm_motion_stack/arm_description/urdf/arm.urdf.xacro`
- Xacro-converted model copy: `ros2_ws/src/arm_motion_stack/arm_description/urdf/original/ARM.model.urdf.xacro`

Only the working xacro copy normalizes mesh filenames to `package://arm_description/meshes/...`. The raw original URDF is preserved unchanged.

## Identified Robot Model

- Robot name: `ARM`
- Base link: `base_link`
- Original terminal link: `J6`
- Added tool/end-effector frame: `tool0`
- Planning group: `single_arm`
- Main joints: `J1`, `J2`, `J3`, `J4`, `J5`, `J6`

No explicit TCP/tool link exists in the exported URDF, so `tool0` is currently a fixed zero-offset child of `J6`.

## Link And Joint Tree

```text
base_link
└── J1 joint, revolute, axis 0 0 1, limit [-3.14, 3.14], effort 0, velocity 0
    └── link J1
        └── J2 joint, revolute, axis 0 0 1, limit [-1.57, 1.57], effort 0, velocity 0
            └── link J2
                └── J3 joint, revolute, axis 0 0 1, limit [-3.14, 3.14], effort 0, velocity 0
                    └── link J3
                        └── J4 joint, revolute, axis 0 0 -1, limit [-1.57, 1.57], effort 0, velocity 0
                            └── link J4
                                └── J5 joint, revolute, axis 0 0 1, limit [-3.14, 3.14], effort 0, velocity 0
                                    └── link J5
                                        └── J6 joint, revolute, axis 0 0 1, limit [-1.57, 1.57], effort 0, velocity 0
                                            └── link J6
                                                └── J6_to_tool0 fixed joint
                                                    └── link tool0
```

## URDF Completeness

- Visual geometry: present for `base_link`, `J1`, `J2`, `J3`, `J4`, `J5`, `J6`.
- Collision geometry: present for `base_link`, `J1`, `J2`, `J3`, `J4`, `J5`, `J6`.
- Inertial data: present for `base_link`, `J1`, `J2`, `J3`, `J4`, `J5`, `J6`.
- Joint effort limits: present but all are `0`; treated as TODO.
- Joint velocity limits: present but all are `0`; MoveIt uses a conservative
  provisional `0.05 rad/s`.
- Joint acceleration limits: absent from URDF; MoveIt uses a conservative
  provisional `0.10 rad/s²`.
- Mesh units: STL files came from SolidWorks. The URDF dimensions appear meter-scale from joint origins, but STL unit metadata is not reliable. Verify mm/m scale in RViz and Isaac Sim before dynamics or collision tuning.

## Packages

- `arm_description`: URDF/xacro, meshes, RViz display.
- `arm_moveit_config`: SRDF, kinematics, OMPL, joint limits, MoveIt launch.
- `arm_control`: ros2_control fake hardware and Isaac topic-based placeholders.
- `arm_bringup`: top-level launch files.
- `arm_kinematics`: FK, IK, Cartesian path demos.
- `arm_dynamics`: placeholder dynamics interface for future Pinocchio.
- `arm_planning_examples`: MoveIt planning examples.
- `arm_interfaces`: custom FK/IK services and trajectory status message.
- `miraculous_driver`: real CANopen hardware plugin, calibrated profile,
  controller configuration, and guarded real-MoveIt launch.

## Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools
```

## Build

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Validate URDF

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
source /opt/ros/humble/setup.bash
ros2 run xacro xacro src/arm_motion_stack/arm_description/urdf/arm.urdf.xacro > /tmp/arm.urdf
check_urdf /tmp/arm.urdf
```

## Launch

Display only:

```bash
ros2 launch arm_bringup display.launch.py
```

Headless display smoke test:

```bash
ROS_LOG_DIR=/tmp/ros-log ros2 launch arm_bringup display.launch.py use_gui:=false use_rviz:=false
```

Fake ros2_control:

```bash
ros2 launch arm_bringup fake_demo.launch.py
```

Headless fake-control smoke test:

```bash
ROS_LOG_DIR=/tmp/ros-log ros2 launch arm_bringup fake_demo.launch.py use_rviz:=false
```

MoveIt demo:

```bash
ros2 launch arm_bringup moveit_demo.launch.py
```

Headless MoveIt smoke test:

```bash
ROS_LOG_DIR=/tmp/ros-log ros2 launch arm_bringup moveit_demo.launch.py use_rviz:=false
```

### Joint State Topic Isolation

The bringup launch files default to `joint_states_topic:=/arm_joint_states`.
This keeps the ARM MoveIt stack from consuming unrelated global `/joint_states`
messages from other robots, stale simulators, or gripper publishers. If you need
the traditional global topic in a clean single-robot session, launch with:

```bash
ros2 launch arm_bringup moveit_demo.launch.py joint_states_topic:=/joint_states
```

MoveIt may still log the internal subscription name as `joint_states`; the
launch remap resolves it to the selected topic.

Isaac Sim backend:

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/ros-log ros2 run arm_isaac_sim arm_isaac_ros_bridge.py
```

In another terminal, start IsaacLab:

```bash
cd /home/alienware/Documents/xlerobot/IsaacLab
source /home/alienware/miniconda3/etc/profile.d/conda.sh
conda activate env_isaaclab
env TERM=xterm PYTHONDONTWRITEBYTECODE=1 ./isaaclab.sh -p \
  /home/alienware/Documents/PersonalProject/ros2_ws/src/arm_motion_stack/arm_isaac_sim/scripts/arm_isaac_backend.py \
  --backend_config /home/alienware/Documents/PersonalProject/ros2_ws/src/arm_motion_stack/arm_isaac_sim/config/arm_isaac_backend.yaml \
  --headless --force_exit
```

Then launch MoveIt/ros2_control against Isaac:

```bash
ROS_LOG_DIR=/tmp/ros-log ros2 launch arm_bringup isaac_moveit.launch.py \
  hardware_type:=isaac use_sim_time:=true use_rviz:=false
```

The true `hardware_type:=isaac` path uses the local `arm_control/IsaacTopicSystem`
ros2_control hardware plugin. IsaacLab runs in Python 3.11, while ROS Humble
`rclpy` is Python 3.10, so the bridge is intentionally split into two processes:

- ROS side: `arm_isaac_ros_bridge.py` translates ROS topics to localhost UDP.
- Isaac side: `arm_isaac_backend.py` translates UDP to IsaacLab articulation commands.
- Joint command topic: `/isaac_joint_commands`
- Joint state topic: `/isaac_joint_states`
- UDP command/state ports: `127.0.0.1:55100` and `127.0.0.1:55101`

The generated Isaac USD lives at:

```text
ros2_ws/src/arm_motion_stack/arm_description/usd/arm_isaac.usd
```

Drive tuning is recorded in `arm_isaac_sim/config/arm_isaac_drives.yaml`.

### Isaac Closed-Loop Validation

After the three Isaac backend terminals above are running, execute a MoveIt
trajectory from a fourth ROS terminal:

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ROS_LOG_DIR=/tmp/ros-log ros2 run arm_planning_examples plan_to_joint_target \
  --ros-args -p execute:=true
```

Check that Isaac is receiving commands and publishing state:

```bash
ros2 topic echo /isaac_joint_commands
ros2 topic echo /isaac_joint_states
ros2 control list_controllers
```

Expected controller state:

```text
joint_state_broadcaster active
arm_controller active
```

Useful diagnostics if execution stalls:

```bash
ros2 topic list | rg 'isaac|arm_controller|joint'
ros2 action list | rg trajectory
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

The closed loop is healthy when `plan_to_joint_target execute:=true` completes
without controller/action errors, `/isaac_joint_commands` changes during
execution, and `/isaac_joint_states` reports changing `J1` through `J6` values
from the IsaacLab backend.

## FK / IK / Planning Demos

Start MoveIt first:

```bash
ros2 launch arm_bringup moveit_demo.launch.py
```

Run FK:

```bash
ros2 run arm_kinematics fk_demo --ros-args \
  -p joint_positions:="[0.0, -0.5, 0.8, 0.0, 0.6, 0.0]"
```

Run IK:

```bash
ros2 run arm_kinematics ik_demo
```

Run Cartesian path demo:

```bash
ros2 run arm_kinematics cartesian_path_demo --ros-args -p execute:=false
```

Run planning examples:

```bash
ros2 run arm_planning_examples plan_to_joint_target --ros-args -p execute:=false
ros2 run arm_planning_examples plan_to_pose_target --ros-args -p execute:=false
ros2 run arm_planning_examples plan_cartesian_path --ros-args -p execute:=false
ros2 run arm_planning_examples execute_named_pose --ros-args -p pose:=ready -p execute:=false
ros2 run arm_planning_examples moveit_joint_smoke_test
```

The FK/IK/planning example nodes auto-load `robot_description`, SRDF, and KDL kinematics parameters from the installed `arm_description` and `arm_moveit_config` packages.

## Latest Validation

After installing the ROS Humble MoveIt/ros2_control dependencies, the following passed:

- `xacro` generation and `check_urdf /tmp/arm.urdf`
- full `colcon build --symlink-install`
- fake ros2_control launch with both controllers activated
- MoveIt demo launch with OMPL ready for planning
- `fk_demo`
- `ik_demo`
- `plan_to_joint_target`
- `plan_to_pose_target`
- `moveit_demo.launch.py use_rviz:=false` after joint-state topic isolation; no stale `joint24` or gripper joint names were reported by MoveIt.
- IsaacLab URDF-to-USD conversion generated `arm_description/usd/arm_isaac.usd`.
- IsaacLab validation passed for USD checks, joint discovery, finite state, and position tracking; max observed validation error was about `0.0013 rad`.
- `arm_isaac_backend.py --duration 2.0` started the IsaacLab UDP backend and exited cleanly.
- `arm_isaac_ros_bridge.py` reached ready state outside the network-restricted sandbox.

In this sandbox, use `ROS_LOG_DIR=/tmp/ros-log` because `~/.ros/log` is read-only.

## Runtime Notes

- MoveIt may print `No 3D sensor plugin(s) defined for octomap updates` when no
  depth camera or point cloud updater is configured. This is expected for the
  current planning-only setup and does not block FK, IK, OMPL planning, or
  ros2_control trajectory execution.
- Do not add placeholder octomap sensor parameters unless a real 3D sensor
  topic exists. In ROS 2 Humble, raw MoveIt `sensors_3d.yaml` list syntax is not
  a valid node parameter file when passed directly to `move_group`.
- When a real RGB-D/depth/point-cloud source is available, add a proper
  occupancy map updater configuration using the real sensor topic and frame.

## Parameters To Replace Before Real Use

- Replace `tool0` zero offset with the real TCP transform.
- Replace all joint `effort=0` values with actuator torque/effort limits.
- Replace all joint `velocity=0` values with real velocity limits.
- Add real acceleration and jerk limits for planning and time parameterization.
- Validate inertial tensors from CAD against the actual assembled robot.
- Consider simplified collision meshes; current collision uses full visual STL meshes.
- Confirm STL unit scaling in RViz and Isaac Sim.

## Dual-Arm Extension

Keep `arm_description` as the single-arm source of truth and add a new wrapper later, for example `dual_arm.urdf.xacro`, that instantiates left/right arms with prefixes. Then add SRDF groups:

- `left_arm`
- `right_arm`
- `dual_arm`

Avoid renaming the current single-arm joints in this package. Use xacro prefixes only in the dual-arm wrapper.

## Pinocchio Extension

`arm_dynamics::DynamicsModel` is a placeholder interface. Replace its implementation with Pinocchio by:

1. Loading the generated URDF from `arm.urdf.xacro`.
2. Building a Pinocchio model with the same joint order `J1..J6`.
3. Replacing gravity, mass matrix, and inverse dynamics placeholder returns.
4. Adding tests that compare dimensions and finite outputs for representative joint states.

## Ruckig Extension

MoveIt currently uses standard time parameterization. Add Ruckig by installing the MoveIt Ruckig smoothing plugin and configuring request adapters/time parameterization after real velocity, acceleration, and jerk limits are known.

## Real Hardware Interface

The current real hardware path is implemented in `miraculous_driver` through the
`miraculous_driver/MiraculousSystem` ros2_control plugin and
`hardware_type:=real`. Use the fail-closed real MoveIt runbook:

```text
ros2_ws/src/arm_motion_stack/docs/MOVEIT_REAL_BRINGUP.md
```

The production profile intentionally ships with `calibrated: false`. After six
reviewed position limits are installed, the real launch still starts both
hardware and trajectory controller inactive; follow the runbook's explicit
activation order.

Keep the controller names and joints stable:

- `joint_state_broadcaster`
- `arm_controller`
- joints: `J1`, `J2`, `J3`, `J4`, `J5`, `J6`
