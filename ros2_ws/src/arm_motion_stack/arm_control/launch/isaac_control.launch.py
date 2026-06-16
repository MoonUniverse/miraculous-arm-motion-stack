import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hardware_type = LaunchConfiguration("hardware_type")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    arm_description_share = get_package_share_directory("arm_description")
    arm_control_share = get_package_share_directory("arm_control")
    robot_xacro = os.path.join(arm_description_share, "urdf", "arm.urdf.xacro")
    controller_config = os.path.join(arm_control_share, "config", "isaac_ros2_controllers.yaml")

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", robot_xacro, " hardware_type:=", hardware_type]),
            value_type=str,
        )
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config, {"use_sim_time": use_sim_time}],
        output="screen",
        remappings=[("joint_states", joint_states_topic)],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("hardware_type", default_value="isaac_mock"),
        DeclareLaunchArgument("joint_states_topic", default_value="/arm_joint_states"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        control_node,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=control_node,
                on_start=[TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner])],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=control_node,
                on_start=[TimerAction(period=5.0, actions=[arm_controller_spawner])],
            )
        ),
    ])
