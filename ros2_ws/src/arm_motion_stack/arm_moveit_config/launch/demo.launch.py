from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    fake_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_control"), "launch", "fake_control.launch.py"])
        ),
        launch_arguments={"joint_states_topic": joint_states_topic}.items(),
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_moveit_config"), "launch", "move_group.launch.py"])
        ),
        launch_arguments={"joint_states_topic": joint_states_topic}.items(),
    )
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_moveit_config"), "launch", "moveit_rviz.launch.py"])
        ),
        launch_arguments={"joint_states_topic": joint_states_topic}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("joint_states_topic", default_value="/arm_joint_states"),
        fake_control,
        move_group,
        moveit_rviz,
    ])
