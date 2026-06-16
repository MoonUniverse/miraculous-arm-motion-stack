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
    robot_xacro = os.path.join(arm_description_share, "urdf", "arm.urdf.xacro")

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", robot_xacro, " hardware_type:=", hardware_type]),
            value_type=str,
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": load_text("arm_moveit_config", "srdf/arm.srdf")
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml("arm_moveit_config", "config/kinematics.yaml")
    }
    joint_limits = {
        "robot_description_planning": load_yaml("arm_moveit_config", "config/joint_limits.yaml")
    }
    ompl_planning = load_yaml("arm_moveit_config", "config/ompl_planning.yaml")
    controllers = load_yaml("arm_moveit_config", "config/controllers.yaml")
    trajectory_execution = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
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
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            ompl_planning,
            controllers,
            trajectory_execution,
            planning_scene_monitor,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("joint_states", joint_states_topic)],
    )

    return LaunchDescription([
        DeclareLaunchArgument("hardware_type", default_value="fake"),
        DeclareLaunchArgument("joint_states_topic", default_value="/arm_joint_states"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        move_group,
    ])
