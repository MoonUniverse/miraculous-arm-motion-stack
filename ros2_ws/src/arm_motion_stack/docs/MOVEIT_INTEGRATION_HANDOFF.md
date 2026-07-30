# MoveIt Integration Handoff

Last updated: 2026-07-30

This document records the context used to implement the first real Miraculous
arm MoveIt integration. The implementation baseline is commit `f298255`;
real-hardware acceptance is still pending calibrated limits. The operator
runbook is `docs/MOVEIT_REAL_BRINGUP.md`.
The architecture and test baseline are recorded in
`docs/MOVEIT_REAL_INTEGRATION_DESIGN.md`.

## Repository State

- Repository root: `/home/alienware/Documents/PersonalProject`
- ROS workspace: `ros2_ws`
- Stack source: `ros2_ws/src/arm_motion_stack`
- Branch at handoff: `main`
- Driver HEAD at handoff: `86e3dff Prevent TPDO feedback lost wakeups`
- SDK checkout: `miraculous_sdk`, clean at upstream commit `33e7a37`
- `isaac_ros_cumotion/` is an unrelated untracked reference tree. Do not stage
  or modify it unless explicitly requested.

The SDK is maintained by the motor team. The driver should adapt to the SDK;
do not add local SDK patches without explicitly documenting and reviewing them.

## Proven Driver Behavior

The latest SDK owns CAN reception through its internal receive thread. The
driver does not actively call `miraculous_motor_poll()` during normal runtime.

The CSP lifecycle is:

1. `MiraculousArm::init()` opens configured motors and performs one-time CSP
   setup.
2. `MiraculousArm::enable_csp()` pauses competing SYNC activity, obtains fresh
   feedback for every configured joint, preloads actual positions as CSP
   targets, latches the complete target group, enables all axes, verifies the
   final states, and only then reports success.
3. Any CSP enable failure enters best-effort arm-wide rollback. Fault reset is
   never performed implicitly.
4. `set_targets_rad()` writes every configured RPDO target before sending one
   shared manual SYNC.
5. `disable()` returns the motors to a non-enabled state without repeating
   one-time CSP initialization.

Manual CSP feedback follows this path:

```text
set_targets_rad()
  -> refresh_feedback_locked()
  -> sync_and_wait_for_fresh_feedback_locked()
  -> miraculous_motor_sync_send()
  -> SDK receive thread reads TPDO2
  -> SDK on_tpdo1() updates its position/velocity cache
  -> MiraculousArm::tpdo_trampoline()
  -> tpdo2_generation_ increments
  -> feedback_cv_ wakes the command thread
```

For J1, SYNC is CAN ID `0x080` and TPDO2 is `0x281`. A feedback timeout means
the driver did not observe a fresh TPDO2 callback before its deadline; it does
not by itself prove that the CAN-wire response exceeded that deadline.

Commit `86e3dff` serializes the TPDO generation update with the condition
variable predicate mutex to prevent a lost wakeup between predicate evaluation
and blocking.

## Proven Trajectory And Teaching Behavior

Real-hardware tests completed successfully:

- Single-motor sinusoidal trajectory tracking.
- Multi-motor synchronized trajectory tracking with one shared SYNC.
- Disabled teach recording while manually moving the joints.
- V2 CSV recording with timestamps, sample indices, and selected joint columns.
- Safe CSP enable and current-position seeding before playback.
- Minimum-jerk approach from the current position to the first CSV point.
- Timestamp-based trajectory playback and final disable.

Playback uses a configurable manual-feedback deadline:

```text
feedback_timeout_ms=10
```

This is a maximum wait, not a fixed per-cycle delay. Feedback arrival wakes the
condition variable immediately. Playback remains fail-fast when a target write,
fresh-feedback wait, or drive fault check fails.

V2 CSV must be identified by its explicit second column:

```csv
timestamp,sample_index,J1,J2
```

The parser checks `sample_index` before legacy column-count matching. The CSV
filename has no joint-selection meaning; every joint named in the header must
also be configured by the playback node.

Relevant commits:

- `088223a Harden CSP enable seeding and rollback`
- `d25ed5c Adapt driver to SDK receive thread`
- `1872eb7 Make playback feedback timeout configurable`
- `a59d913 Fix V2 playback CSV header detection`
- `86e3dff Prevent TPDO feedback lost wakeups`

Detailed driver references:

- `miraculous_driver/docs/IMPLEMENTATION_DETAILS.md`
- `miraculous_driver/docs/TEACH_HARDWARE_TEST_GUIDE.md`
- `miraculous_driver/docs/TRAJECTORY_TEST_DOCUMENTATION.md`

## Existing MoveIt And ros2_control Surface

The MoveIt integration is not starting from an empty repository. Inspect and
validate these existing components before adding new ones:

- Robot model: `arm_description/urdf/arm.urdf.xacro`
- ros2_control model: `arm_description/urdf/ros2_control.xacro`
- Hardware plugin:
  - `miraculous_driver/include/miraculous_driver/miraculous_system.hpp`
  - `miraculous_driver/src/miraculous_system.cpp`
- Real controller configuration:
  `miraculous_driver/config/real_ros2_controllers.yaml`
- Real launch entry:
  `miraculous_driver/launch/moveit_real.launch.py`
- MoveIt package: `arm_moveit_config`
- SRDF: `arm_moveit_config/srdf/arm.srdf`
- Kinematics: `arm_moveit_config/config/kinematics.yaml`
- MoveIt joint limits: `arm_moveit_config/config/joint_limits.yaml`
- MoveIt controller mapping:
  `arm_moveit_config/config/moveit_controllers.yaml`
- Versioned real-arm profile:
  `miraculous_driver/config/real_arm_profile.yaml`

The intended real-hardware command path is:

```text
MoveIt
  -> FollowJointTrajectory
  -> joint_trajectory_controller
  -> controller_manager
  -> MiraculousSystem::write()
  -> MiraculousArm::set_targets_rad()
  -> SDK/CANopen
```

State feedback should return through:

```text
TPDO2
  -> MiraculousArm feedback cache
  -> MiraculousSystem::read()
  -> ros2_control state interfaces
  -> joint_state_broadcaster
  -> MoveIt current state monitor
```

Do not route MoveIt execution through `playback_node`; that node is a dedicated
CSV teaching tool, not the ros2_control hardware interface.

## Implemented MoveIt Audit Outcomes

The implementation now:

- refuses real MoveIt launch until all six reviewed limits are present and
  `calibrated: true`;
- starts `ARMSystem` and `arm_controller` inactive for explicit operator
  activation;
- rejects malformed/non-finite parameters and unsafe encoder seeds;
- treats command, feedback freshness, EMCY, and drive faults as latched
  quick-stop failures that require process restart;
- uses closed-loop JointTrajectoryController feedback and conservative
  provisional velocity/acceleration limits;
- provides `moveit_joint_smoke_test` for a guarded current-relative joint step;
- keeps `moveit_controllers.yaml` as the only MoveIt controller mapping.

The remaining milestone is staged real-hardware acceptance using
`MOVEIT_REAL_BRINGUP.md`.

Offline validation completed with a clean isolated build, 36 passing tests,
real-xacro `check_urdf`, fail-closed launch rejection for the uncalibrated
profile, and successful guarded fake planning/execution. During the fake
launch's final `Ctrl-C`, MoveIt 2.5.9 produced a class-loader teardown
segmentation fault after controller_manager had already deactivated controllers
and shut down the hardware successfully. Track that MoveGroup teardown issue
separately; do not confuse it with a trajectory or hardware-stop failure.

## Original Audit Checklist

Perform this audit before editing:

1. Read `MiraculousSystem::on_init()`, `on_configure()`, `on_activate()`,
   `on_deactivate()`, `read()`, and `write()` end to end.
2. Confirm URDF, SRDF, controller YAML, hardware plugin, and MoveIt use exactly
   the same joint names and order.
3. Confirm command/state interface types match
   `joint_trajectory_controller` requirements.
4. Confirm `on_activate()` seeds command interfaces from fresh actual
   positions before the first controller write.
5. Confirm controller update frequency is compatible with manual SYNC and the
   TPDO feedback deadline.
6. Trace stop, cancel, controller deactivation, process shutdown, EMCY, and
   driver fault behavior to `disable()` or `quick_stop()` as appropriate.
7. Validate software limits across URDF, MoveIt, ros2_control, and driver
   configuration. Do not assume one layer protects all others.
8. Check launch sequencing: robot description, controller manager, hardware
   activation, broadcasters, trajectory controller, move_group, then RViz or
   planning clients.
9. Treat `No 3D sensor plugin(s) defined for octomap updates` as expected for
   planning without a 3D sensor. Do not add a placeholder sensor configuration.

## Low-Risk Bringup Order

1. Build the affected packages and run all driver tests without hardware.
2. Start `ros2_control` without MoveIt and verify controllers and joint states.
3. Verify activation produces no commanded-position jump.
4. Send a small single-joint trajectory through
   `joint_trajectory_controller`.
5. Test trajectory cancellation and controller deactivation.
6. Repeat with two joints.
7. Start MoveIt in plan-only mode and compare the current robot state.
8. Execute a small single-joint MoveIt plan.
9. Expand to coordinated multi-joint motion only after command, feedback,
   limits, cancellation, and fault handling pass.

## New Codex Session Prompt

Start a new session from the repository root and say:

```text
先不要改代码。阅读
ros2_ws/src/arm_motion_stack/docs/MOVEIT_INTEGRATION_HANDOFF.md，
然后检查当前 Git 状态，并完整审查现有 MoveIt、ros2_control、
MiraculousSystem 和 launch 链路。先输出真实工作流程、问题清单和分阶段
真机接入计划，等我确认后再修改。
```

Useful initial commands:

```bash
cd /home/alienware/Documents/PersonalProject
git status --short --branch
git log -5 --oneline
git -C miraculous_sdk status --short --branch
colcon list --base-paths ros2_ws/src/arm_motion_stack
```
