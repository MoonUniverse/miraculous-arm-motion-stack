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
    input_file = LaunchConfiguration("input_file")
    speed_scale = LaunchConfiguration("speed_scale")
    loop = LaunchConfiguration("loop")
    approach_velocity_rad_s = LaunchConfiguration("approach_velocity_rad_s")
    approach_rate_hz = LaunchConfiguration("approach_rate_hz")
    approach_min_duration_s = LaunchConfiguration("approach_min_duration_s")
    start_tolerance_rad = LaunchConfiguration("start_tolerance_rad")
    feedback_timeout_ms = LaunchConfiguration("feedback_timeout_ms")
    max_command_step_rad = LaunchConfiguration("max_command_step_rad")
    max_following_error_rad = LaunchConfiguration("max_following_error_rad")
    following_error_cycles = LaunchConfiguration("following_error_cycles")

    arm_description_share = get_package_share_directory("arm_description")
    robot_xacro = os.path.join(arm_description_share, "urdf", "arm.urdf.xacro")

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", robot_xacro, " hardware_type:=fake"]),
            value_type=str,
        )
    }

    playback_node = Node(
        package="miraculous_driver",
        executable="playback_node",
        name="playback",
        output="screen",
        parameters=[{
            "can_interface": can_interface,
            "baudrate": baudrate,
            "node_ids": ParameterValue(node_ids, value_type=str),
            "joint_indices": ParameterValue(joint_indices, value_type=str),
            "position_min": ParameterValue(position_min, value_type=str),
            "position_max": ParameterValue(position_max, value_type=str),
            "joint_states_topic": joint_states_topic,
            "input_file": ParameterValue(input_file, value_type=str),
            "speed_scale": speed_scale,
            "loop": loop,
            "approach_velocity_rad_s": approach_velocity_rad_s,
            "approach_rate_hz": approach_rate_hz,
            "approach_min_duration_s": approach_min_duration_s,
            "start_tolerance_rad": start_tolerance_rad,
            "feedback_timeout_ms": feedback_timeout_ms,
            "max_command_step_rad": max_command_step_rad,
            "max_following_error_rad": max_following_error_rad,
            "following_error_cycles": following_error_cycles,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("joint_states_topic", default_value="/arm_joint_states"),
        DeclareLaunchArgument("can_interface", default_value="can1"),
        DeclareLaunchArgument("baudrate", default_value="0"),
        DeclareLaunchArgument("node_ids", default_value="1,2,3,4,5,6"),
        DeclareLaunchArgument("joint_indices", default_value="0,1,2,3,4,5",
                              description="ROS joint indices for node_ids: 0=J1..5=J6"),
        DeclareLaunchArgument("position_min", default_value="0.0,0.0,0.0,0.0,0.0,0.0"),
        DeclareLaunchArgument("position_max", default_value="0.0,0.0,0.0,0.0,0.0,0.0"),
        DeclareLaunchArgument("input_file", default_value=""),
        DeclareLaunchArgument("speed_scale", default_value="1.0",
                              description="Playback time scale in (0, 1]; values above 1 are rejected"),
        DeclareLaunchArgument("loop", default_value="false"),
        DeclareLaunchArgument("approach_velocity_rad_s", default_value="0.1"),
        DeclareLaunchArgument("approach_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("approach_min_duration_s", default_value="0.5"),
        DeclareLaunchArgument("start_tolerance_rad", default_value="0.005"),
        DeclareLaunchArgument("feedback_timeout_ms", default_value="15"),
        DeclareLaunchArgument("max_command_step_rad", default_value="0.005",
                              description="Maximum accepted per-cycle joint target change"),
        DeclareLaunchArgument("max_following_error_rad", default_value="0.05",
                              description="Hardware-layer commanded/actual error limit"),
        DeclareLaunchArgument("following_error_cycles", default_value="3",
                              description="Fresh feedback cycles allowed above the error limit"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
            remappings=[("joint_states", joint_states_topic)],
        ),
        playback_node,
    ])
