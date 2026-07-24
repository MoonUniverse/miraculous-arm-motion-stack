import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    can_interface = LaunchConfiguration("can_interface")
    baudrate = LaunchConfiguration("baudrate")
    node_ids = LaunchConfiguration("node_ids")
    joint_indices = LaunchConfiguration("joint_indices")
    position_min = LaunchConfiguration("position_min")
    position_max = LaunchConfiguration("position_max")
    sync_period_us = LaunchConfiguration("sync_period_us")
    amplitude = LaunchConfiguration("amplitude")
    period = LaunchConfiguration("period")
    frequency = LaunchConfiguration("frequency")
    waveform = LaunchConfiguration("waveform")
    test_joint = LaunchConfiguration("test_joint")
    test_joints = LaunchConfiguration("test_joints")
    duration = LaunchConfiguration("duration")
    output_file = LaunchConfiguration("output_file")
    settle_time = LaunchConfiguration("settle_time")
    enable_emcy_monitor = LaunchConfiguration("enable_emcy_monitor")

    arm_description_share = get_package_share_directory("arm_description")
    robot_xacro = os.path.join(arm_description_share, "urdf", "arm.urdf.xacro")

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", robot_xacro, " hardware_type:=fake"]),
            value_type=str,
        )
    }

    test_node = Node(
        package="miraculous_driver",
        executable="trajectory_tracking_test_node",
        name="trajectory_test",
        output="screen",
        parameters=[{
            "can_interface": can_interface,
            "baudrate": baudrate,
            "node_ids": ParameterValue(node_ids, value_type=str),
            "joint_indices": ParameterValue(joint_indices, value_type=str),
            "position_min": ParameterValue(position_min, value_type=str),
            "position_max": ParameterValue(position_max, value_type=str),
            "sync_period_us": sync_period_us,
            "joint_states_topic": joint_states_topic,
            "amplitude": amplitude,
            "period": period,
            "frequency": frequency,
            "waveform": waveform,
            "test_joint": test_joint,
            "test_joints": ParameterValue(test_joints, value_type=str),
            "duration": duration,
            "output_file": output_file,
            "settle_time": settle_time,
            "enable_emcy_monitor": ParameterValue(enable_emcy_monitor, value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("joint_states_topic", default_value="/arm_joint_states"),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("baudrate", default_value="1000"),
        DeclareLaunchArgument("node_ids", default_value="1,2,3,4,5,6"),
        DeclareLaunchArgument("joint_indices", default_value="0,1,2,3,4,5",
                              description="ROS joint indices for node_ids: 0=J1..5=J6"),
        DeclareLaunchArgument("position_min", default_value="0.0,0.0,0.0,0.0,0.0,0.0"),
        DeclareLaunchArgument("position_max", default_value="0.0,0.0,0.0,0.0,0.0,0.0"),
        DeclareLaunchArgument("sync_period_us", default_value="10000",
                              description="CSP SYNC period [us]. 0=manual, >0=SDK timer"),
        DeclareLaunchArgument("amplitude", default_value="0.03",
                              description="Sine/cosine amplitude [rad]"),
        DeclareLaunchArgument("period", default_value="6.0",
                              description="Sine/cosine period [s]"),
        DeclareLaunchArgument("frequency", default_value="100.0",
                              description="Command update rate [Hz]"),
        DeclareLaunchArgument("waveform", default_value="sin",
                              description="Waveform: 'sin' or 'cos'"),
        DeclareLaunchArgument("test_joint", default_value="0",
                              description="Legacy single joint index 0=J1..5=J6"),
        DeclareLaunchArgument("test_joints", default_value="",
                              description="Comma-separated synchronized test joints; "
                                          "empty uses test_joint"),
        DeclareLaunchArgument("duration", default_value="3.0",
                              description="Auto-stop after N seconds (0=manual)"),
        DeclareLaunchArgument("output_file", default_value="",
                              description="CSV output path (empty=auto timestamp)"),
        DeclareLaunchArgument("settle_time", default_value="0.5",
                              description="Settling time before test starts [s]"),
        DeclareLaunchArgument("enable_emcy_monitor", default_value="true",
                              description="Register the SDK's dedicated EMCY callback "
                                          "(motor fault reporting)"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
            remappings=[("joint_states", joint_states_topic)],
        ),
        test_node,
    ])
