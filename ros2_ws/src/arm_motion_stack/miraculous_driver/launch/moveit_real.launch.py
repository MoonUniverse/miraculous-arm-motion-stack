import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    can_interface = LaunchConfiguration("can_interface")
    baudrate = LaunchConfiguration("baudrate")
    node_ids = LaunchConfiguration("node_ids")
    joint_indices = LaunchConfiguration("joint_indices")
    position_min = LaunchConfiguration("position_min")
    position_max = LaunchConfiguration("position_max")
    sync_period_us = LaunchConfiguration("sync_period_us")
    read_rate_hz = LaunchConfiguration("read_rate_hz")
    arm_description_share = get_package_share_directory("arm_description")
    robot_xacro = os.path.join(arm_description_share, "urdf", "arm.urdf.xacro")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ", robot_xacro,
                " hardware_type:=real",
                " can_interface:=", can_interface,
                " baudrate:=", baudrate,
                " node_ids:=", node_ids,
                " joint_indices:=", joint_indices,
                " position_min:=", position_min,
                " position_max:=", position_max,
                " sync_period_us:=", sync_period_us,
                " read_rate_hz:=", read_rate_hz,
            ]),
            value_type=str,
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("joint_states_topic", default_value="/arm_joint_states"),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("baudrate", default_value="1000"),
        DeclareLaunchArgument("node_ids", default_value="1,2,3,4,5,6"),
        DeclareLaunchArgument("joint_indices", default_value="0,1,2,3,4,5",
                              description="ROS joint indices for node_ids: 0=J1..5=J6"),
        DeclareLaunchArgument("position_min", default_value="0.0,0.0,0.0,0.0,0.0,0.0"),
        DeclareLaunchArgument("position_max", default_value="0.0,0.0,0.0,0.0,0.0,0.0"),
        DeclareLaunchArgument("sync_period_us", default_value="0"),
        DeclareLaunchArgument("read_rate_hz", default_value="100.0"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
            remappings=[("joint_states", joint_states_topic)],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("miraculous_driver"), "launch", "real_control.launch.py")
            ),
            launch_arguments={
                "joint_states_topic": joint_states_topic,
                "can_interface": can_interface,
                "baudrate": baudrate,
                "node_ids": node_ids,
                "joint_indices": joint_indices,
                "position_min": position_min,
                "position_max": position_max,
                "sync_period_us": sync_period_us,
                "read_rate_hz": read_rate_hz,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("arm_moveit_config"), "launch", "move_group.launch.py")
            ),
            launch_arguments={"joint_states_topic": joint_states_topic}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("arm_moveit_config"), "launch", "moveit_rviz.launch.py")
            ),
            condition=IfCondition(use_rviz),
        ),
    ])
