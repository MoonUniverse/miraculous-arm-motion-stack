import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def load_text(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return file.read()


def generate_launch_description():
    hardware_type = LaunchConfiguration("hardware_type")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    arm_description_share = get_package_share_directory("arm_description")
    moveit_config_share = get_package_share_directory("arm_moveit_config")
    robot_xacro = os.path.join(arm_description_share, "urdf", "arm.urdf.xacro")
    rviz_config = os.path.join(moveit_config_share, "config", "moveit.rviz")

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", robot_xacro, " hardware_type:=", hardware_type]),
            value_type=str,
        )
    }
    parameters = [
        robot_description,
        {"robot_description_semantic": load_text("arm_moveit_config", "srdf/arm.srdf")},
        {"robot_description_kinematics": load_yaml("arm_moveit_config", "config/kinematics.yaml")},
        {"robot_description_planning": load_yaml("arm_moveit_config", "config/joint_limits.yaml")},
        load_yaml("arm_moveit_config", "config/ompl_planning.yaml"),
        load_yaml("arm_moveit_config", "config/controllers.yaml"),
        {"use_sim_time": use_sim_time},
    ]

    return LaunchDescription([
        DeclareLaunchArgument("hardware_type", default_value="fake"),
        DeclareLaunchArgument("joint_states_topic", default_value="/arm_joint_states"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="rviz2",
            executable="rviz2",
            name="moveit_rviz",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=parameters,
            remappings=[("joint_states", joint_states_topic)],
        ),
    ])
