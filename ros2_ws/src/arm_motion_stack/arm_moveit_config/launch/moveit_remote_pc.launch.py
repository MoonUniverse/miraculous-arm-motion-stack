"""PC-side MoveIt and RViz for a controller hosted on the robot board."""

import os

import yaml
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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_yaml(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _load_text(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, "r", encoding="utf-8") as stream:
        return stream.read()


def _launch_setup(context):
    profile_path = LaunchConfiguration("real_profile").perform(context)
    profile = load_real_arm_profile(profile_path, require_calibrated=True)
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    heartbeat_topic = LaunchConfiguration("heartbeat_topic")
    heartbeat_topic_value = heartbeat_topic.perform(context)

    description_share = get_package_share_directory("arm_description")
    moveit_share = get_package_share_directory("arm_moveit_config")
    robot_xacro = os.path.join(description_share, "urdf", "arm.urdf.xacro")
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
    shared_parameters = [
        robot_description,
        {
            "robot_description_semantic": _load_text(
                "arm_moveit_config", "srdf/arm.srdf"
            )
        },
        {
            "robot_description_kinematics": _load_yaml(
                "arm_moveit_config", "config/kinematics.yaml"
            )
        },
        {"robot_description_planning": profile.moveit_joint_limits},
        _load_yaml("arm_moveit_config", "config/ompl_planning.yaml"),
        _load_yaml("arm_moveit_config", "config/moveit_controllers.yaml"),
        {"use_sim_time": False},
    ]
    trajectory_execution = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.02,
    }
    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": False,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=shared_parameters
        + [trajectory_execution, planning_scene_monitor],
        remappings=[("joint_states", joint_states_topic)],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=["-d", os.path.join(moveit_share, "config", "moveit.rviz")],
        parameters=shared_parameters,
        remappings=[("joint_states", joint_states_topic)],
    )
    heartbeat = Node(
        package="arm_remote_control",
        executable="remote_pc_heartbeat",
        output="screen",
        parameters=[
            {
                "heartbeat_topic": heartbeat_topic,
                "profile_fingerprint": profile.fingerprint,
                "heartbeat_period_ms": profile.remote_heartbeat_period_ms,
            }
        ],
    )

    shutdown_if_critical_process_exits = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=process,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason=f"critical PC process exited: {name}")
                    )
                ],
            )
        )
        for process, name in ((move_group, "move_group"), (heartbeat, "heartbeat"))
    ]

    return [
        LogInfo(msg=f"PC real-arm profile fingerprint sha256:{profile.fingerprint}"),
        heartbeat,
        move_group,
        rviz,
        *shutdown_if_critical_process_exits,
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
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "joint_states_topic", default_value="/arm_joint_states"
            ),
            DeclareLaunchArgument(
                "heartbeat_topic", default_value="/arm_remote_control/heartbeat"
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
