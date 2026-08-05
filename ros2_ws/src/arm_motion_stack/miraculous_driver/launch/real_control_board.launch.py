"""Fail-closed board-side control for the complete six-axis real arm.

This launch owns the local trajectory interpolation and hardware I/O.  It does
not start MoveIt or RViz, so the board can be built without the MoveIt stack.
"""

import os

from ament_index_python.packages import get_package_share_directory
from arm_real_config import load_real_arm_profile, real_xacro_command_arguments
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context):
    profile_path = LaunchConfiguration("real_profile").perform(context)
    profile = load_real_arm_profile(profile_path, require_calibrated=True)
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    heartbeat_topic = LaunchConfiguration("heartbeat_topic")
    heartbeat_topic_value = heartbeat_topic.perform(context)

    description_share = get_package_share_directory("arm_description")
    driver_share = get_package_share_directory("miraculous_driver")
    robot_xacro = os.path.join(description_share, "urdf", "arm.urdf.xacro")
    controller_config = os.path.join(
        driver_share, "config", "real_ros2_controllers.yaml"
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                real_xacro_command_arguments(
                    robot_xacro,
                    profile,
                    remote_heartbeat_topic=heartbeat_topic_value,
                )
            ),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
        remappings=[("joint_states", joint_states_topic)],
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            controller_config,
            {
                "update_rate": profile.controller_update_rate_hz,
                "hardware_components_initial_state": {"inactive": ["ARMSystem"]},
                "use_sim_time": False,
            },
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("joint_states", joint_states_topic),
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "--inactive",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )
    remote_motion_watchdog = Node(
        package="arm_remote_control",
        executable="remote_motion_watchdog",
        output="screen",
        parameters=[
            {
                "heartbeat_topic": heartbeat_topic,
                "expected_profile_fingerprint": profile.fingerprint,
                "soft_timeout_ms": profile.remote_soft_timeout_ms,
                "monitor_period_ms": 20,
                "cancel_hold_ms": 60,
                "controller_name": "arm_controller",
                "controller_manager_name": "/controller_manager",
                "trajectory_action": "/arm_controller/follow_joint_trajectory",
            }
        ],
    )

    return [
        LogInfo(
            msg=f"BOARD real-arm profile fingerprint sha256:{profile.fingerprint}"
        ),
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        remote_motion_watchdog,
        RegisterEventHandler(
            OnProcessExit(
                target_action=remote_motion_watchdog,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason="board remote-motion watchdog exited"
                        )
                    )
                ],
            )
        ),
    ]


def generate_launch_description():
    default_profile = os.path.join(
        get_package_share_directory("arm_real_config"),
        "config",
        "real_arm_profile.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("real_profile", default_value=default_profile),
            DeclareLaunchArgument(
                "joint_states_topic", default_value="/arm_joint_states"
            ),
            DeclareLaunchArgument(
                "heartbeat_topic", default_value="/arm_remote_control/heartbeat"
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
