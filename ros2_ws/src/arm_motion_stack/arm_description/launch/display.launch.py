from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration("use_gui")
    model = PathJoinSubstitution([FindPackageShare("arm_description"), "urdf", "arm.urdf.xacro"])
    rviz_config = PathJoinSubstitution([FindPackageShare("arm_description"), "rviz", "display.rviz"])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", model, " hardware_type:=fake"]),
            value_type=str,
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument("use_gui", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=None,
            parameters=[{"use_gui": use_gui}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
