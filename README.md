# PersonalProject

This repository contains the existing ARM URDF export and the new ROS 2 motion stack built around it.

## Main Contents

- `arm_description/`: original SolidWorks-to-URDF export as received.
- `ros2_ws/src/arm_motion_stack/`: ROS 2 + MoveIt 2 + ros2_control + Isaac Sim-ready project generated from the existing URDF.
- `miraculous_sdk/`: existing SDK files in the workspace.
- `CHANGELOG.md`: durable record of repository setup and major project changes.

## Current ROS 2 Stack

The active ROS 2 project is:

```text
ros2_ws/src/arm_motion_stack
```

Start from its README for build, launch, URDF analysis, MoveIt configuration, ros2_control setup, and future extension notes:

```text
ros2_ws/src/arm_motion_stack/README.md
```

Detailed implementation notes, launch topology, MoveIt troubleshooting, joint-state isolation, and future extension details are recorded in:

```text
ros2_ws/src/arm_motion_stack/docs/IMPLEMENTATION_DETAILS.md
```

## Git Notes

Generated colcon directories are ignored:

- `ros2_ws/build/`
- `ros2_ws/install/`
- `ros2_ws/log/`

Local editor state is also ignored through `.gitignore`.
