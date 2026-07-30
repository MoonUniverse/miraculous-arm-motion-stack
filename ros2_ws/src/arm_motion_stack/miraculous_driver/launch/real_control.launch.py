"""Lower-level ros2_control bringup.

Unlike moveit_real.launch.py this launch keeps subset mapping available for
commissioning.  It therefore does not claim to be the production MoveIt path.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    argument_names = (
        "joint_states_topic",
        "can_interface",
        "baudrate",
        "encoder_bw",
        "reduction_ratio",
        "node_ids",
        "joint_indices",
        "position_min",
        "position_max",
        "sync_period_us",
        "read_rate_hz",
        "state_poll_rate_hz",
        "manual_feedback_timeout_ms",
        "feedback_stale_timeout_ms",
        "enable_emcy_monitor",
        "require_full_arm",
        "require_position_limits",
        "update_rate",
    )
    values = {name: LaunchConfiguration(name) for name in argument_names}

    description_share = get_package_share_directory("arm_description")
    driver_share = get_package_share_directory("miraculous_driver")
    robot_xacro = os.path.join(description_share, "urdf", "arm.urdf.xacro")
    controller_config = os.path.join(
        driver_share, "config", "real_ros2_controllers.yaml"
    )
    xacro_arguments = [
        "xacro ",
        robot_xacro,
        " hardware_type:=real",
    ]
    for name in argument_names[1:-1]:
        xacro_arguments.extend([f" {name}:=", values[name]])
    robot_description = {
        "robot_description": ParameterValue(
            Command(xacro_arguments),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        remappings=[("joint_states", values["joint_states_topic"])],
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config, {"update_rate": values["update_rate"]}],
        output="screen",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("joint_states", values["joint_states_topic"]),
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    declarations = [
        DeclareLaunchArgument("joint_states_topic", default_value="/arm_joint_states"),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("baudrate", default_value="1000"),
        DeclareLaunchArgument("encoder_bw", default_value="19"),
        DeclareLaunchArgument("reduction_ratio", default_value="100.0"),
        DeclareLaunchArgument("node_ids", default_value="1,2,3,4,5,6"),
        DeclareLaunchArgument("joint_indices", default_value="0,1,2,3,4,5"),
        DeclareLaunchArgument(
            "position_min", default_value="0.0,0.0,0.0,0.0,0.0,0.0"
        ),
        DeclareLaunchArgument(
            "position_max", default_value="0.0,0.0,0.0,0.0,0.0,0.0"
        ),
        DeclareLaunchArgument("sync_period_us", default_value="0"),
        DeclareLaunchArgument("read_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("state_poll_rate_hz", default_value="0.0"),
        DeclareLaunchArgument("manual_feedback_timeout_ms", default_value="5"),
        DeclareLaunchArgument("feedback_stale_timeout_ms", default_value="30"),
        DeclareLaunchArgument("enable_emcy_monitor", default_value="true"),
        DeclareLaunchArgument("require_full_arm", default_value="false"),
        DeclareLaunchArgument("require_position_limits", default_value="false"),
        DeclareLaunchArgument("update_rate", default_value="100"),
    ]
    return LaunchDescription(
        declarations
        + [
            robot_state_publisher,
            control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
        ]
    )
