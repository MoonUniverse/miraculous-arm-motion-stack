# Changelog

## 2026-08-05

### Split Real-Hardware Board / MoveIt PC Deployment

- Added hardware-independent `arm_real_config`; it is now the single source for
  the calibrated real-arm profile, strict validation, MoveIt limits, shared
  real-xacro arguments, and a deterministic deployment fingerprint.
- Added `miraculous_driver/real_control_board.launch.py` for board-local
  robot_state_publisher, ros2_control, JTC, Driver, SDK, and CAN ownership.
- Added `arm_moveit_config/moveit_remote_pc.launch.py` for remote MoveGroup and
  RViz without a Miraculous SDK dependency.
- Both launch paths fail closed on an uncalibrated profile and print the same
  SHA-256 profile fingerprint for pre-activation comparison.
- Explicitly selected
  `moveit_simple_controller_manager/MoveItSimpleControllerManager` in the
  MoveIt controller configuration.
- Added deployment-contract tests and a dual-host build, DDS, bring-up, and
  validation runbook in `docs/REMOTE_MOVEIT_DEPLOYMENT.md`.
- Kept `miraculous_driver/moveit_real.launch.py` as the single-host compatibility
  path; production deployment uses the two new entry points.
- Added hardware-independent `arm_remote_control`: the PC publishes a
  profile-bound volatile heartbeat, and the board cancels the active trajectory
  then deactivates JTC when that heartbeat times out.
- Added a second, independent heartbeat guard inside `MiraculousSystem`.
  Hardware activation now requires a fresh matching heartbeat; if the soft stop
  does not settle the arm before the hard timeout, the existing verified
  CiA402 Quick Stop path is used.
- Lost-PC faults remain latched after reconnection and require board-local reset
  plus explicit controller reactivation.

## 2026-07-06

### Miraculous SDK Consolidation

- Removed older SDK snapshots:
  - `miraculous_sdk/`
  - `miraculous_sdk_x86_64/`
- Kept the current SDK snapshot as the default x86-64 driver dependency:
  - `miraculous_sdk_x86_64_linux_gnu_20260702/`
- `miraculous_driver` now defaults to the 2026-07-02 SDK snapshot and requires `-DMIRACULOUS_SDK_DIR=/abs/path/to/sdk` for any non-default SDK or target architecture.
- Latest SDK `_ex` position APIs are treated as joint/load-side radians; the ROS driver no longer exposes `reduction_ratio`.
- `node_ids` can now list only the mounted motors, and `joint_indices` maps those motors into ROS slots `0=J1 ... 5=J6` for partial-arm bring-up.

## 2026-06-30

### Real Hardware First-Stage Safety Gate

- Added a real hardware bring-up runbook:
  - `ros2_ws/src/arm_motion_stack/miraculous_driver/docs/REAL_HARDWARE_BRINGUP.md`
- Exposed real hardware parameters through `arm.urdf.xacro` and `ros2_control.xacro`:
  - `can_interface`
  - `baudrate`
  - `node_ids`
  - `position_min`
  - `position_max`
  - `sync_period_us`
  - `read_rate_hz`
- Hardened real motion paths:
  - `MiraculousSystem`, `trajectory_tracking_test_node`, and `playback_node` validate `position_min` / `position_max` before motion.
  - Motion-capable paths now pass `position_min` / `position_max` into `JointConfig` so `MiraculousArm::check_limits()` can clamp commanded targets.
  - `trajectory_tracking_test_node` defaults are now conservative for first bring-up: `amplitude:=0.03`, `period:=6.0`, `duration:=3.0`.
- Updated real hardware launch files to pass the same parameter set to `robot_state_publisher` and `ros2_control_node`.
- Updated documentation paths from `/home/alienware/Desktop/PersonalProject` to `/home/alienware/Documents/PersonalProject`.

## 2026-06-22

### miraculous_driver ROS 2 Package

- Added `miraculous_driver` package wrapping the `miraculous_sdk` (CANopen/CiA402) for the ARM 6DOF manipulator.
- Core library `MiraculousArm` (C++ wrapper):
  - Manages 6 `MiraMotor` handles on a single CAN bus (node IDs 1–6).
  - Lifecycle: `init()` (open + bootstrap, not enabled), `init_passive()` (teach mode, motors de-energized), `enable_csp()`, `disable()`, `quick_stop()`, `fault_reset()`, `shutdown()`.
  - Background read thread at configurable rate (default 100 Hz) polls position/velocity/state via mutex-cached buffers.
  - CSP write: `csp_set_target` per motor with `manual=true` (SDK does not auto-send SYNC), followed by a single unified SYNC broadcast (CAN ID 0x80).
  - Unit conversion: `rad = pulse / pulses_per_radian` per joint.
  - EMCY detection via CAN receive callback (filters 0x081–0x0FF).
- ros2_control hardware plugin `miraculous_driver/MiraculousSystem`:
  - Implements `hardware_interface::SystemInterface` (on_init/on_configure/on_activate/on_deactivate/on_cleanup/read/write).
  - Seeds position commands from real encoder readings on configure to prevent jump on activate.
  - Registered via `pluginlib_export_plugin_description_file`.
- `ros2_control.xacro` updated with `hardware_type == 'real'` branch.
- Teach/record node (`teach_record_node`):
  - Opens motors in passive mode (free to drag).
  - Publishes `/arm_joint_states` (sensor_msgs/JointState).
  - Records 6 joint angles to CSV at configurable rate (default 50 Hz).
  - Services: `/teach/start`, `/teach/stop`.
- Playback node (`playback_node`):
  - Loads CSV trajectory and replays via CSP mode at recorded rate.
  - Services: `/playback/play`, `/playback/stop`.
  - Supports `speed_scale` and `loop` parameters.
- Launch files: `real_control.launch.py`, `moveit_real.launch.py`, `teach.launch.py`, `playback.launch.py`.
- Config files: `real_ros2_controllers.yaml`, `miraculous_arm_params.yaml` (software-limit placeholders for user to fill).
- CMakeLists defaults to the current 2026-07-02 x86-64 SDK snapshot: `miraculous_sdk_x86_64_linux_gnu_20260702`. Other SDK builds must be supplied explicitly with `-DMIRACULOUS_SDK_DIR=/abs/path/to/sdk`.
- Build verified:

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select miraculous_driver --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

- Plugin discovery confirmed; executables registered as `miraculous_driver teach_record_node` and `miraculous_driver playback_node`.
- Next steps: fill in real software-limit values in `miraculous_arm_params.yaml`; test on real CAN bus.

## 2026-06-16

### IsaacLab Backend Integration

- Added generated Isaac USD asset:
  - `ros2_ws/src/arm_motion_stack/arm_description/usd/arm_isaac.usd`
- Added `arm_isaac_sim` package with:
  - IsaacLab backend config and drive tuning YAML.
  - USD drive patch script.
  - IsaacLab validation script.
  - IsaacLab UDP backend.
  - ROS Humble UDP bridge node.
- Added local `arm_control/IsaacTopicSystem` ros2_control hardware plugin.
  - Publishes `/isaac_joint_commands` as `sensor_msgs/msg/JointState`.
  - Reads `/isaac_joint_states` as `sensor_msgs/msg/JointState`.
  - Keeps `J1..J6` order stable for the existing `arm_controller`.
- Reworked Isaac launch path so `hardware_type:=isaac` is no longer an external placeholder.
- Split Isaac/ROS bridging into two processes because `env_isaaclab` uses Python 3.11 while ROS Humble `rclpy` is Python 3.10.

Validation:

- Converted `/tmp/arm_isaac_abs.urdf` to `arm_isaac.usd` with IsaacLab `convert_urdf.py`.
- Patched drive gains for `/ARM/joints/J1..J6`; cleaned a stale DriveAPI initially applied to same-name link `/ARM/J1`.
- `validate_arm_isaaclab.py --headless --force_exit --steps 160` passed with max tracking error about `0.0013 rad`.
- `arm_isaac_backend.py --duration 2.0` started the IsaacLab UDP backend and exited cleanly.
- `arm_isaac_ros_bridge.py` reached ready state outside the network-restricted sandbox.
- Full workspace build passed with system Python:

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

## 2026-06-15

### Repository Initialization

- Initialized `/home/alienware/Documents/PersonalProject` as a Git repository.
- Renamed the initial branch to `main`.
- Added `.gitignore` to keep generated colcon output, editor state, logs, and binary runtime artifacts out of version control.
- Added this root changelog and a root README so future work has a stable entry point.

### arm_motion_stack Bootstrap

- Created `ros2_ws/src/arm_motion_stack` as a ROS 2 workspace project around the existing ARM URDF.
- Preserved the original model at `arm_description/urdf/original/ARM.urdf`.
- Added ROS 2 model entry point `arm_description/urdf/arm.urdf.xacro`.
- Added xacro wrapper files:
  - `arm_description/urdf/materials.xacro`
  - `arm_description/urdf/ros2_control.xacro`
  - `arm_description/urdf/original/ARM.model.urdf.xacro`
- Normalized working mesh references to `package://arm_description/meshes/...` while keeping the raw original URDF unchanged.
- Added a temporary fixed `tool0` frame as a zero-offset child of `J6`; this is marked TODO for replacement with the real TCP transform.

### URDF Analysis Recorded

- Robot name: `ARM`
- Base link: `base_link`
- Original terminal link: `J6`
- MoveIt tool/end-effector frame: `tool0`
- Planning group: `single_arm`
- Main joints: `J1`, `J2`, `J3`, `J4`, `J5`, `J6`
- Link tree: `base_link -> J1 -> J2 -> J3 -> J4 -> J5 -> J6 -> tool0`
- Visual, collision, and inertial elements are present for all original links.
- Joint position limits exist in the original URDF.
- Joint effort and velocity limits exist but are all `0`; MoveIt configuration uses temporary test values with TODO comments.
- Joint acceleration limits are not present in URDF; temporary MoveIt test values were added with TODO comments.

### ROS 2 Packages Added

- `arm_description`: xacro, meshes, display launch, RViz config.
- `arm_moveit_config`: SRDF, KDL IK config, OMPL config, controller config, initial positions, MoveIt launch files.
- `arm_control`: fake hardware ros2_control config and Isaac Sim topic-based placeholder config.
- `arm_bringup`: top-level launch files for display, fake control, MoveIt demo, and Isaac MoveIt mode.
- `arm_kinematics`: FK, IK, and Cartesian path MoveIt demos.
- `arm_dynamics`: placeholder dynamics interface for future Pinocchio integration.
- `arm_planning_examples`: MoveIt planning examples for joint target, pose target, Cartesian path, and named pose execution.
- `arm_interfaces`: `ComputeIK`, `ComputeFK`, and `JointTrajectoryStatus` interface definitions.

### Validation Notes

- XML/package/SRDF/xacro files were statically checked with `xmllint`.
- These packages were successfully built in the current environment:
  - `arm_description`
  - `arm_interfaces`
  - `arm_dynamics`
  - `arm_control`
  - `arm_moveit_config`
  - `arm_bringup`
- Full build is currently blocked by missing system dependencies, especially `moveit_ros_planning_interface`.
- `xacro` and full MoveIt/ros2_control/RViz runtime dependencies were not installed in the current ROS Humble environment, so `check_urdf` and launch validation could not be completed yet.
- The ROS build should be run with system Python to avoid Miniconda interference:

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

### Next Required Steps

- Install missing ROS Humble dependencies listed in `ros2_ws/src/arm_motion_stack/README.md`.
- Re-run xacro and `check_urdf`.
- Re-run full `colcon build --symlink-install`.
- Validate:
  - `ros2 launch arm_bringup display.launch.py`
  - `ros2 launch arm_bringup fake_demo.launch.py`
  - `ros2 launch arm_bringup moveit_demo.launch.py`
- Replace temporary joint velocity, acceleration, effort, and TCP values with real robot parameters.

### Dependency Installation And Validation Follow-Up

- Installed the missing ROS Humble dependencies with apt:
  - `ros-humble-moveit`
  - `ros-humble-ros2-control`
  - `ros-humble-ros2-controllers`
  - `ros-humble-joint-state-publisher-gui`
  - `ros-humble-xacro`
  - `ros-humble-rviz2`
  - `ros-humble-tf2-ros`
  - `ros-humble-tf2-tools`
- Re-ran URDF generation and validation:

```bash
ros2 run xacro xacro src/arm_motion_stack/arm_description/urdf/arm.urdf.xacro > /tmp/arm.urdf
check_urdf /tmp/arm.urdf
```

- `check_urdf` parsed successfully and reported the expected tree:
  `base_link -> J1 -> J2 -> J3 -> J4 -> J5 -> J6 -> tool0`.
- Full workspace build passed:

```bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

- Added headless launch switches while preserving GUI/RViz defaults:
  - `use_gui:=false` for `arm_bringup display.launch.py`
  - `use_rviz:=false` for display/fake/moveit/isaac bringup launch files
- Validated `fake_demo.launch.py use_rviz:=false`:
  - `ARMSystem` fake hardware initialized, configured, and activated.
  - `arm_controller` loaded, configured, and activated.
  - `joint_state_broadcaster` loaded, configured, and activated.
- Validated `moveit_demo.launch.py use_rviz:=false`:
  - MoveGroup loaded robot model `ARM`.
  - OMPL planning pipeline initialized.
  - MoveGroup reached `You can start planning now!`.
  - `arm_controller` and `joint_state_broadcaster` activated.
- Updated Isaac bringup behavior:
  - `isaac_moveit.launch.py` now defaults to `hardware_type:=isaac_mock`, which uses `mock_components/GenericSystem` while preserving the Isaac bringup path.
  - The real `hardware_type:=isaac` path still references `topic_based_ros2_control/TopicBasedSystem` and requires that external plugin to be installed.
- Validated `isaac_moveit.launch.py use_rviz:=false` in default `isaac_mock` mode:
  - `ARMSystem` initialized through `mock_components/GenericSystem`.
  - `arm_controller` and `joint_state_broadcaster` activated.
  - MoveGroup reached `You can start planning now!`.
- Validated planning examples:
  - `plan_to_joint_target` plan-only succeeded.
  - `plan_to_pose_target` plan-only succeeded.
- Validated kinematics examples:
  - `fk_demo` succeeded and produced a `tool0` pose.
  - `ik_demo` succeeded after changing its default target to a reachable FK-derived pose.

### Code Fixes From Validation

- Removed duplicate parameter auto-declaration in MoveIt demo nodes.
- Added helper utilities in `arm_kinematics` and `arm_planning_examples` so `ros2 run` demos can auto-load:
  - generated `robot_description` from `arm.urdf.xacro`
  - `robot_description_semantic` from `arm.srdf`
  - KDL kinematics parameters for `single_arm`
- Reworked `fk_demo` and `ik_demo` to use MoveIt `RobotModelLoader` and `RobotState` directly, avoiding unnecessary dependency on the MoveGroup action server for local FK/IK tests.
- Updated default IK and pose-planning targets to the reachable all-zero-joint FK pose:
  - position `[0.020061, 0.000397, 0.549780]`
  - orientation xyzw `[0.500000, 0.500000, 0.500002, 0.499998]`

### Remaining Runtime Notes

- In the Codex sandbox, DDS reports `getifaddrs: Operation not permitted` and FastDDS UDP socket warnings because network access is restricted. The tested nodes still reached the relevant ready/success states.
- Launch commands in this environment need `ROS_LOG_DIR=/tmp/ros-log` because `~/.ros/log` is read-only in the sandbox.
- MoveIt reports `No 3D sensor plugin(s) defined for octomap updates`; this is expected for the current no-depth-sensor configuration and does not block planning.
- KDL warns that `base_link` has inertial data; this comes from the original SolidWorks URDF. Add a dummy root/world link later if KDL root inertial handling becomes important.

### MoveIt Joint State Topic Isolation

- Investigated MoveIt startup errors where joints such as `joint24`, `joint25`, `right_gripper_*`, and `left_gripper_*` were reported as missing from robot model `ARM`.
- Confirmed those joint names were not present in the generated URDF, SRDF, MoveIt joint limits, controller config, or installed package configuration for this stack.
- Root cause: MoveIt was consuming the global `/joint_states` topic, which can contain stale or unrelated joint states from other robot stacks or simulator sessions.
- Added `joint_states_topic` launch argument with default `/arm_joint_states` across:
  - `arm_description/launch/display.launch.py`
  - `arm_control/launch/fake_control.launch.py`
  - `arm_control/launch/isaac_control.launch.py`
  - `arm_bringup/launch/display.launch.py`
  - `arm_bringup/launch/fake_demo.launch.py`
  - `arm_bringup/launch/moveit_demo.launch.py`
  - `arm_bringup/launch/isaac_moveit.launch.py`
  - `arm_moveit_config/launch/demo.launch.py`
  - `arm_moveit_config/launch/move_group.launch.py`
  - `arm_moveit_config/launch/moveit_rviz.launch.py`
- Remapped `joint_states` for robot state publisher, ros2_control, MoveGroup, and MoveIt RViz to keep this stack isolated by default.
- Removed an experimental placeholder `sensors_3d.yaml` load because raw MoveIt sensor-list YAML is not a valid ROS 2 node parameter file when passed directly to `move_group` in Humble.
- Rebuilt the workspace successfully with system Python:

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

- Re-tested `moveit_demo.launch.py use_rviz:=false`; MoveGroup reached `You can start planning now!` and no stale `joint24`/gripper missing-joint errors were reported.
- Confirmed `/arm_joint_states` is present during bringup. A global `/joint_states` may still exist if other publishers are running, so the isolated default remains intentional.
- Note: MoveIt logs may still display the internal subscription name `joint_states`; the launch remap resolves it to `joint_states_topic`.
- Remaining octomap log: `No 3D sensor plugin(s) defined for octomap updates`. This is expected until a real depth camera or point-cloud updater is configured and does not block the current planning stack.

### Detailed Implementation Documentation

- Added `ros2_ws/src/arm_motion_stack/docs/IMPLEMENTATION_DETAILS.md`.
- Documented package responsibilities, URDF/xacro strategy, MoveIt configuration, ros2_control configuration, bringup launch topology, joint-state topic isolation, octomap runtime note, validation commands, and future extension paths for Isaac Sim, Pinocchio, Ruckig, dual-arm support, and real hardware.
