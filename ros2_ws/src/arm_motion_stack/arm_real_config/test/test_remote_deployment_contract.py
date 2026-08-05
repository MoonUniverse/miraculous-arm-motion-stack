from pathlib import Path

import yaml


STACK_ROOT = Path(__file__).parents[2]
BOARD_LAUNCH = (
    STACK_ROOT / "miraculous_driver" / "launch" / "real_control_board.launch.py"
)
PC_LAUNCH = (
    STACK_ROOT / "arm_moveit_config" / "launch" / "moveit_remote_pc.launch.py"
)
WATCHDOG_SOURCE = (
    STACK_ROOT
    / "arm_remote_control"
    / "src"
    / "remote_motion_watchdog.cpp"
)
HARD_GUARD_SOURCE = (
    STACK_ROOT
    / "miraculous_driver"
    / "src"
    / "miraculous_system.cpp"
)


def test_board_launch_owns_control_but_not_planning_or_rviz():
    source = BOARD_LAUNCH.read_text(encoding="utf-8")
    assert 'package="controller_manager"' in source
    assert 'executable="ros2_control_node"' in source
    assert '"arm_controller"' in source
    assert 'package="arm_remote_control"' in source
    assert 'executable="remote_motion_watchdog"' in source
    assert 'package="moveit_ros_move_group"' not in source
    assert 'package="rviz2"' not in source


def test_pc_launch_owns_planning_but_not_hardware_control():
    source = PC_LAUNCH.read_text(encoding="utf-8")
    assert 'package="moveit_ros_move_group"' in source
    assert 'package="rviz2"' in source
    assert 'package="arm_remote_control"' in source
    assert 'executable="remote_pc_heartbeat"' in source
    assert 'package="controller_manager"' not in source
    assert "miraculous_driver" not in source


def test_both_launches_use_the_shared_validated_profile_and_xacro_arguments():
    for launch_path in (BOARD_LAUNCH, PC_LAUNCH):
        source = launch_path.read_text(encoding="utf-8")
        assert "get_package_share_directory(\"arm_real_config\")" in source
        assert "load_real_arm_profile(profile_path, require_calibrated=True)" in source
        assert "real_xacro_command_arguments(" in source
        assert "remote_heartbeat_topic=heartbeat_topic_value" in source


def test_lost_pc_cancels_and_deactivates_before_hardware_quick_stop_fallback():
    watchdog = WATCHDOG_SOURCE.read_text(encoding="utf-8")
    hard_guard = HARD_GUARD_SOURCE.read_text(encoding="utf-8")

    assert "async_cancel_all_goals" in watchdog
    assert "deactivate_controllers = {controller_name_}" in watchdog
    assert "SwitchController::Request::STRICT" in watchdog
    assert "kFaultLatched" in watchdog
    assert "remote_heartbeat_is_fresh" in hard_guard
    assert "remote_command_advanced" in hard_guard
    assert "position_commands_[index] - last_sent_commands_[index]" in hard_guard
    assert "external PC heartbeat exceeded the hard timeout" in hard_guard
    assert "fail_safe_stop" in hard_guard


def test_remote_supervision_package_is_independent_of_motor_sdk():
    cmake = (STACK_ROOT / "arm_remote_control" / "CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    assert "miraculous_sdk" not in cmake
    assert "miraculous_driver" not in cmake


def test_moveit_controller_manager_plugin_is_explicit():
    config_path = (
        STACK_ROOT
        / "arm_moveit_config"
        / "config"
        / "moveit_controllers.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    assert config["moveit_controller_manager"] == (
        "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    )
    assert config["moveit_simple_controller_manager"]["controller_names"] == [
        "arm_controller"
    ]


def test_there_is_only_one_real_arm_profile_source():
    profile_paths = sorted(STACK_ROOT.glob("*/config/real_arm_profile.yaml"))
    assert profile_paths == [
        STACK_ROOT / "arm_real_config" / "config" / "real_arm_profile.yaml"
    ]
