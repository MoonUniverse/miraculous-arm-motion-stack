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
export ARM_REDUCTION_RATIO="REAL_J1,REAL_J2,REAL_J3,REAL_J4,REAL_J5,REAL_J6"
export ARM_LIMIT_MIN="MIN_J1,MIN_J2,MIN_J3,MIN_J4,MIN_J5,MIN_J6"
export ARM_LIMIT_MAX="MAX_J1,MAX_J2,MAX_J3,MAX_J4,MAX_J5,MAX_J6"
```

- `ARM_REDUCTION_RATIO`: motor revolutions per joint/load revolution for each joint. ROS commands/states stay joint-side radians; the SDK position APIs use motor-side radians.
- `ARM_LIMIT_MIN` / `ARM_LIMIT_MAX`: conservative software limits in radians.
- `0.0,0.0` for a joint disables clamping and should only be used for passive readout.

## 3. Bring Up CAN

```bash
sudo ip link set can0 down || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

## 4. Read-Only Encoder Check

This starts passive teach mode. Motors are opened/bootstraped for encoder reads but not enabled.

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/ros-log ros2 launch miraculous_driver teach.launch.py \
  can_interface:=can0 \
  reduction_ratio:="$ARM_REDUCTION_RATIO" \
  auto_record:=false
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/alienware/Documents/PersonalProject/ros2_ws/install/setup.bash
ros2 topic echo /arm_joint_states
```

Pass criteria:

- `/arm_joint_states` publishes `J1` through `J6`.
- Dragging each joint changes the expected joint value.
- The sign and approximate magnitude are explainable before enabling CSP.
- No EMCY/fault messages appear.

Optional recording:

```bash
ros2 service call /teach_record/start std_srvs/srv/Trigger
ros2 service call /teach_record/stop std_srvs/srv/Trigger
```

## 5. Single-Joint CSP Test

Start with J1 only, short duration, small amplitude:

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/ros-log ros2 launch miraculous_driver trajectory_test.launch.py \
  can_interface:=can0 \
  reduction_ratio:="$ARM_REDUCTION_RATIO" \
  position_min:="$ARM_LIMIT_MIN" \
  position_max:="$ARM_LIMIT_MAX" \
  test_joint:=0 \
  amplitude:=0.03 \
  period:=6.0 \
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
- No EMCY/fault messages appear.

Repeat one joint at a time: `test_joint:=1` through `test_joint:=5`.

## 6. Stop Path

For normal stop:

```bash
ros2 service call /trajectory_test/stop std_srvs/srv/Trigger
```

If the process is unhealthy, stop the launch process and power down according to the bench safety procedure. The C++ destructors call `disable()` and `shutdown()`, but hardware power should still be treated as the final authority.

## 7. Deferred Until This Passes

- Full `moveit_real.launch.py` execution.
- Multi-joint playback.
- Large-amplitude trajectory tracking.
- cuMotion/GPU planning integration.
