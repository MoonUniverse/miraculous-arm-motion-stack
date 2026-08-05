# Real Hardware Bring-Up Runbook

This runbook is for the first real ARM bring-up stage: CAN/encoder readout first, then one small single-joint CSP tracking test. Do not run full MoveIt execution until this stage passes.

## 1. Build

Use system Python for ROS 2 Humble:

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select miraculous_driver arm_description \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## 2. Required Real Parameters

Set these before any motion test:

```bash
export ARM_LIMIT_MIN="MIN_J1,MIN_J2,MIN_J3,MIN_J4,MIN_J5,MIN_J6"
export ARM_LIMIT_MAX="MAX_J1,MAX_J2,MAX_J3,MAX_J4,MAX_J5,MAX_J6"
```

- SDK `_ex` position APIs already use joint/load-side radians. The driver forwards `encoder_bw` / `reduction_ratio` hardware params to the SDK (defaults 19 / 100.0 = SDK defaults); only change them for a different motor model.
- `ARM_LIMIT_MIN` / `ARM_LIMIT_MAX`: conservative software limits in radians.
- `0.0,0.0` may represent an unconfigured limit only in passive readout. Active
  playback and trajectory nodes reject missing or unordered limits.
- `node_ids` and `joint_indices` must have the same length. `joint_indices` uses ROS slots `0=J1 ... 5=J6`.

Examples:

```bash
# Only J1 mounted: CAN node 1 drives ROS J1.
node_ids:=1 joint_indices:=0

# J1-J3 mounted.
node_ids:=1,2,3 joint_indices:=0,1,2

# Only J3 mounted: CAN node 3 drives ROS J3.
node_ids:=3 joint_indices:=2 test_joint:=2
```

## 3. Bring Up CAN

The tested baseline is `can1` with `baudrate:=0`, meaning the SDK preserves the
existing SocketCAN configuration. Configure the interface through the approved
bench procedure, then verify it before starting ROS:

```bash
ip -details link show can1
```

## 4. Read-Only Encoder Check

This starts passive teach mode. Every configured drive is opened/bootstraped,
commanded with `miraculous_motor_shutdown()` (controlword `0x0006`), and
verified in `Ready to Switch On`
before the node reports ready. The teach path does not configure CSP or start
the arm background read thread.

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/ros-log ros2 launch miraculous_driver teach.launch.py \
  can_interface:=can1 \
  baudrate:=0 \
  node_ids:=1,2,3,4,5,6 \
  joint_indices:=0,1,2,3,4,5 \
  record_rate:=50.0 \
  feedback_timeout_ms:=15 \
  max_consecutive_misses:=10 \
  output_file:=/tmp/teach_v2.csv \
  auto_record:=false
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/alienware/Documents/PersonalProject/ros2_ws/install/setup.bash
ros2 topic echo /arm_joint_states
```

Pass criteria:

- The startup log confirms all configured drives are `Ready to Switch On`.
- `/arm_joint_states` publishes exactly the configured joints.
- Dragging each joint changes the expected joint value.
- The sign and approximate magnitude are explainable before enabling CSP.
- No EMCY/fault messages appear.
- `candump` shows one broadcast SYNC per acquisition cycle followed by one
  TPDO2 from every configured node.

Optional recording:

```bash
ros2 service call /teach_record/start std_srvs/srv/Trigger
ros2 service call /teach_record/stop std_srvs/srv/Trigger
```

The V2 CSV header is `timestamp,sample_index,<configured joints>`, for example
`timestamp,sample_index,J1,J3`. A row is written only after both configured
nodes return fresh TPDO2 feedback. A gap in `sample_index` records a missed
acquisition. Playback accepts this V2 joint mapping and first moves from the
current pose to the recorded first point with a velocity-limited minimum-jerk
transition before starting the recorded timestamps.

## 5. Single-Joint CSP Test

Start with J1 only, short duration, small amplitude:

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/ros-log ros2 launch miraculous_driver trajectory_test.launch.py \
  can_interface:=can1 \
  baudrate:=0 \
  node_ids:=1 \
  joint_indices:=0 \
  position_min:="$ARM_LIMIT_MIN" \
  position_max:="$ARM_LIMIT_MAX" \
  test_joint:=0 \
  amplitude:=0.03 \
  period:=6.0 \
  frequency:=50.0 \
  sync_period_us:=0 \
  feedback_timeout_ms:=15 \
  duration:=3.0
```

Start and stop from another terminal:

```bash
ros2 service call /trajectory_test/start std_srvs/srv/Trigger
ros2 service call /trajectory_test/stop std_srvs/srv/Trigger
```

Pass criteria:

- The node enables CSP without a startup jump.
- Only the selected joint follows the command; other joints hold current position.
- The test auto-stops after `duration` and disables the motors.
- The CSV shows finite command/actual/error values with no large discontinuity.
- A target step over `0.005 rad` or tracking error over `0.05 rad` for three
  fresh feedback cycles stops and quarantines the test.
- No EMCY/fault messages appear.

Repeat one joint at a time: `test_joint:=1` through `test_joint:=5`.

## 6. Stop Path

For normal stop:

```bash
ros2 service call /trajectory_test/stop std_srvs/srv/Trigger
```

If the process is unhealthy, stop the launch process and power down according to the bench safety procedure. The C++ destructors call `disable()` and `shutdown()`, but hardware power should still be treated as the final authority.

## 7. Deferred Until This Passes

- Full board-side `real_control_board.launch.py` plus remote-PC
  `moveit_remote_pc.launch.py` execution.
- Multi-joint playback.
- Large-amplitude trajectory tracking.
- cuMotion/GPU planning integration.
