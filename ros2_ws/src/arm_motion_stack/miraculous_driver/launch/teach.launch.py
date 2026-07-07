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
    record_rate = LaunchConfiguration("record_rate")
    auto_record = LaunchConfiguration("auto_record")

    arm_description_share = get_package_share_directory("arm_description")
    robot_xacro = os.path.join(arm_description_share, "urdf", "arm.urdf.xacro")

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", robot_xacro, " hardware_type:=fake"]),
            value_type=str,
        )
    }

    teach_node = Node(
        package="miraculous_driver",
        executable="teach_record_node",
        name="teach_record",
        output="screen",
        parameters=[{
            "can_interface": can_interface,
            "baudrate": baudrate,
            "node_ids": ParameterValue(node_ids, value_type=str),
            "joint_indices": ParameterValue(joint_indices, value_type=str),
            "joint_states_topic": joint_states_topic,
            "record_rate": record_rate,
            "auto_record": auto_record,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("joint_states_topic", default_value="/arm_joint_states"),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("baudrate", default_value="1000"),
        DeclareLaunchArgument("node_ids", default_value="1,2,3,4,5,6"),
        DeclareLaunchArgument("joint_indices", default_value="0,1,2,3,4,5",
                              description="ROS joint indices for node_ids: 0=J1..5=J6"),
        DeclareLaunchArgument("record_rate", default_value="50.0"),
        DeclareLaunchArgument("auto_record", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
            remappings=[("joint_states", joint_states_topic)],
        ),
        teach_node,
    ])
