from pathlib import Path

import pytest
import yaml

from arm_real_config.real_arm_profile import (
    JOINT_NAMES,
    load_real_arm_profile,
    real_xacro_command_arguments,
)


def _valid_profile():
    return {
        "schema_version": 1,
        "calibrated": True,
        "hardware": {
            "can_interface": "can1",
            "baudrate": 0,
            "encoder_bw": 19,
            "reduction_ratio": 100.0,
            "sync_period_us": 0,
            "controller_update_rate_hz": 50,
            "read_rate_hz": 50.0,
            "state_poll_rate_hz": 0.0,
            "manual_feedback_timeout_ms": 15,
            "feedback_stale_timeout_ms": 30,
            "enable_emcy_monitor": True,
            "max_command_step_rad": 0.005,
            "max_following_error_rad": 0.05,
            "following_error_cycles": 3,
            "remote_heartbeat_period_ms": 50,
            "remote_soft_timeout_ms": 250,
            "remote_hard_timeout_ms": 500,
            "remote_stop_velocity_threshold_rad_s": 0.02,
        },
        "joints": {
            name: {
                "node_id": index,
                "position_min": -1.0,
                "position_max": 1.0,
                "max_velocity": 0.05,
                "max_acceleration": 0.10,
            }
            for index, name in enumerate(JOINT_NAMES, start=1)
        },
    }


def _write(tmp_path: Path, profile) -> str:
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.safe_dump(profile), encoding="utf-8")
    return str(path)


def test_valid_profile_drives_xacro_and_moveit_limits(tmp_path):
    profile = load_real_arm_profile(_write(tmp_path, _valid_profile()))
    assert profile.node_ids_csv == "1,2,3,4,5,6"
    assert profile.joint_indices_csv == "0,1,2,3,4,5"
    assert profile.position_min_csv == "-1.0,-1.0,-1.0,-1.0,-1.0,-1.0"
    assert profile.moveit_joint_limits["joint_limits"]["J6"]["max_velocity"] == 0.05
    assert len(profile.fingerprint) == 64
    assert profile.fingerprint == load_real_arm_profile(
        _write(tmp_path, _valid_profile())
    ).fingerprint


def test_real_xacro_arguments_are_derived_from_the_profile(tmp_path):
    profile = load_real_arm_profile(_write(tmp_path, _valid_profile()))
    arguments = real_xacro_command_arguments(
        "/tmp/arm.urdf.xacro",
        profile,
        remote_heartbeat_topic="/robot_1/remote_heartbeat",
    )
    command = "".join(arguments)
    assert "hardware_type:=real" in command
    assert "node_ids:=1,2,3,4,5,6" in command
    assert "position_min:=-1.0,-1.0,-1.0,-1.0,-1.0,-1.0" in command
    assert "remote_heartbeat_topic:=/robot_1/remote_heartbeat" in command
    assert f"remote_profile_fingerprint:={profile.fingerprint}" in command
    assert "remote_watchdog_timeout_ms:=500" in command
    assert "require_full_arm:=true" in command


def test_zero_baudrate_keeps_existing_socketcan_configuration(tmp_path):
    raw_profile = _valid_profile()
    raw_profile["hardware"]["baudrate"] = 0
    profile = load_real_arm_profile(_write(tmp_path, raw_profile))
    assert profile.baudrate == 0


def test_timer_sync_is_rejected_for_real_moveit(tmp_path):
    raw_profile = _valid_profile()
    raw_profile["hardware"]["sync_period_us"] = 10_000
    with pytest.raises(ValueError, match="must be 0"):
        load_real_arm_profile(_write(tmp_path, raw_profile))


def test_production_profile_preserves_tested_bus_baseline():
    path = Path(__file__).parents[1] / "config" / "real_arm_profile.yaml"
    profile = load_real_arm_profile(str(path), require_calibrated=False)
    assert profile.can_interface == "can1"
    assert profile.baudrate == 0
    assert profile.controller_update_rate_hz == 50
    assert profile.read_rate_hz == 50.0
    assert profile.manual_feedback_timeout_ms == 15
    assert profile.sync_period_us == 0
    assert profile.enable_emcy_monitor is True
    assert profile.max_command_step_rad == 0.005
    assert profile.max_following_error_rad == 0.05
    assert profile.following_error_cycles == 3
    assert profile.remote_heartbeat_period_ms == 50
    assert profile.remote_soft_timeout_ms == 250
    assert profile.remote_hard_timeout_ms == 500
    assert profile.remote_stop_velocity_threshold_rad_s == 0.02


def test_production_template_is_rejected_for_real_motion():
    path = Path(__file__).parents[1] / "config" / "real_arm_profile.yaml"
    with pytest.raises(ValueError, match="not calibrated"):
        load_real_arm_profile(str(path))


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda p: p["joints"]["J6"].update(node_id=5), "duplicate"),
        (lambda p: p["joints"]["J1"].update(position_min=2.0), "less than"),
        (lambda p: p["joints"]["J2"].update(position_max=2.0), "exceed URDF"),
        (lambda p: p["hardware"].update(baudrate=-1), "non-negative"),
        (lambda p: p["hardware"].update(encoder_bw=32), r"\[1, 31\]"),
        (lambda p: p["hardware"].update(feedback_stale_timeout_ms=5), "greater"),
        (lambda p: p["hardware"].update(read_rate_hz=float("nan")), "finite"),
        (lambda p: p["hardware"].update(enable_emcy_monitor=False), "must be true"),
        (lambda p: p["hardware"].update(max_command_step_rad=0.0), "greater than zero"),
        (lambda p: p["hardware"].update(following_error_cycles=0), "greater than zero"),
        (
            lambda p: p["hardware"].update(remote_soft_timeout_ms=50),
            "greater than the heartbeat period",
        ),
        (
            lambda p: p["hardware"].update(remote_hard_timeout_ms=200),
            "greater than the soft timeout",
        ),
        (
            lambda p: p["hardware"].update(
                remote_stop_velocity_threshold_rad_s=0.0
            ),
            "greater than zero",
        ),
    ],
)
def test_invalid_profiles_fail_closed(tmp_path, mutate, message):
    profile = _valid_profile()
    mutate(profile)
    with pytest.raises(ValueError, match=message):
        load_real_arm_profile(_write(tmp_path, profile))
