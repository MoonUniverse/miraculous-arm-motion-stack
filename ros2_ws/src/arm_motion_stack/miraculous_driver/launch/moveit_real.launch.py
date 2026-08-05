"""Fail-closed MoveIt bringup for the complete six-axis real arm."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from miraculous_driver.real_arm_profile import load_real_arm_profile


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

    description_share = get_package_share_directory("arm_description")
    moveit_share = get_package_share_directory("arm_moveit_config")
    driver_share = get_package_share_directory("miraculous_driver")
    robot_xacro = os.path.join(description_share, "urdf", "arm.urdf.xacro")
    controller_config = os.path.join(
        driver_share, "config", "real_ros2_controllers.yaml"
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    robot_xacro,
                    " hardware_type:=real",
                    " can_interface:=",
                    profile.can_interface,
                    " baudrate:=",
                    str(profile.baudrate),
                    " encoder_bw:=",
                    str(profile.encoder_bw),
                    " reduction_ratio:=",
                    str(profile.reduction_ratio),
                    " node_ids:=",
                    profile.node_ids_csv,
                    " joint_indices:=",
                    profile.joint_indices_csv,
                    " position_min:=",
                    profile.position_min_csv,
                    " position_max:=",
                    profile.position_max_csv,
                    " sync_period_us:=",
                    str(profile.sync_period_us),
                    " read_rate_hz:=",
                    str(profile.read_rate_hz),
                    " state_poll_rate_hz:=",
                    str(profile.state_poll_rate_hz),
                    " manual_feedback_timeout_ms:=",
                    str(profile.manual_feedback_timeout_ms),
                    " feedback_stale_timeout_ms:=",
                    str(profile.feedback_stale_timeout_ms),
                    " enable_emcy_monitor:=",
                    str(profile.enable_emcy_monitor).lower(),
                    " max_command_step_rad:=",
                    str(profile.max_command_step_rad),
                    " max_following_error_rad:=",
                    str(profile.max_following_error_rad),
                    " following_error_cycles:=",
                    str(profile.following_error_cycles),
                    " require_full_arm:=true",
                    " require_position_limits:=true",
                ]
            ),
            value_type=str,
        )
    }
    semantic = {
        "robot_description_semantic": _load_text(
            "arm_moveit_config", "srdf/arm.srdf"
        )
    }
    kinematics = {
        "robot_description_kinematics": _load_yaml(
            "arm_moveit_config", "config/kinematics.yaml"
        )
    }
    planning_limits = {
        "robot_description_planning": profile.moveit_joint_limits
    }
    ompl = _load_yaml("arm_moveit_config", "config/ompl_planning.yaml")
    moveit_controllers = _load_yaml(
        "arm_moveit_config", "config/moveit_controllers.yaml"
    )
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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
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

    shared_moveit_parameters = [
        robot_description,
        semantic,
        kinematics,
        planning_limits,
        ompl,
        moveit_controllers,
        {"use_sim_time": False},
    ]
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=shared_moveit_parameters
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
        parameters=shared_moveit_parameters,
        remappings=[("joint_states", joint_states_topic)],
    )

    return [
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[move_group, rviz],
            )
        ),
    ]


def generate_launch_description():
    default_profile = os.path.join(
        get_package_share_directory("miraculous_driver"),
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
            OpaqueFunction(function=_launch_setup),
        ]
    )
