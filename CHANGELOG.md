# Changelog

## 2026-06-15

### Repository Initialization

- Initialized `/home/alienware/Desktop/PersonalProject` as a Git repository.
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
cd /home/alienware/Desktop/PersonalProject/ros2_ws
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
