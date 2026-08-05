"""Load and strictly validate the version-controlled real-arm profile."""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Dict, Mapping, Optional

import yaml


JOINT_NAMES = tuple(f"J{index}" for index in range(1, 7))
URDF_POSITION_LIMITS = {
    "J1": (-3.14, 3.14),
    "J2": (-1.57, 1.57),
    "J3": (-3.14, 3.14),
    "J4": (-1.57, 1.57),
    "J5": (-3.14, 3.14),
    "J6": (-1.57, 1.57),
}


@dataclass(frozen=True)
class RealJointConfig:
    node_id: int
    position_min: float
    position_max: float
    max_velocity: float
    max_acceleration: float


@dataclass(frozen=True)
class RealArmProfile:
    calibrated: bool
    can_interface: str
    baudrate: int
    encoder_bw: int
    reduction_ratio: float
    sync_period_us: int
    controller_update_rate_hz: int
    read_rate_hz: float
    state_poll_rate_hz: float
    manual_feedback_timeout_ms: int
    feedback_stale_timeout_ms: int
    enable_emcy_monitor: bool
    max_command_step_rad: float
    max_following_error_rad: float
    following_error_cycles: int
    remote_heartbeat_period_ms: int
    remote_soft_timeout_ms: int
    remote_hard_timeout_ms: int
    remote_stop_velocity_threshold_rad_s: float
    joints: Mapping[str, RealJointConfig]

    def csv(self, field: str) -> str:
        return ",".join(str(getattr(self.joints[name], field)) for name in JOINT_NAMES)

    @property
    def node_ids_csv(self) -> str:
        return self.csv("node_id")

    @property
    def joint_indices_csv(self) -> str:
        return "0,1,2,3,4,5"

    @property
    def position_min_csv(self) -> str:
        return self.csv("position_min")

    @property
    def position_max_csv(self) -> str:
        return self.csv("position_max")

    @property
    def moveit_joint_limits(self) -> Dict[str, Dict[str, Any]]:
        return {
            "joint_limits": {
                name: {
                    "has_position_limits": True,
                    "min_position": joint.position_min,
                    "max_position": joint.position_max,
                    "has_velocity_limits": True,
                    "max_velocity": joint.max_velocity,
                    "has_acceleration_limits": True,
                    "max_acceleration": joint.max_acceleration,
                }
                for name, joint in self.joints.items()
            }
        }

    @property
    def fingerprint(self) -> str:
        """Return a deterministic fingerprint for comparing two deployments."""

        canonical = {
            "calibrated": self.calibrated,
            "hardware": {
                "can_interface": self.can_interface,
                "baudrate": self.baudrate,
                "encoder_bw": self.encoder_bw,
                "reduction_ratio": self.reduction_ratio,
                "sync_period_us": self.sync_period_us,
                "controller_update_rate_hz": self.controller_update_rate_hz,
                "read_rate_hz": self.read_rate_hz,
                "state_poll_rate_hz": self.state_poll_rate_hz,
                "manual_feedback_timeout_ms": self.manual_feedback_timeout_ms,
                "feedback_stale_timeout_ms": self.feedback_stale_timeout_ms,
                "enable_emcy_monitor": self.enable_emcy_monitor,
                "max_command_step_rad": self.max_command_step_rad,
                "max_following_error_rad": self.max_following_error_rad,
                "following_error_cycles": self.following_error_cycles,
                "remote_heartbeat_period_ms": self.remote_heartbeat_period_ms,
                "remote_soft_timeout_ms": self.remote_soft_timeout_ms,
                "remote_hard_timeout_ms": self.remote_hard_timeout_ms,
                "remote_stop_velocity_threshold_rad_s": (
                    self.remote_stop_velocity_threshold_rad_s
                ),
            },
            "joints": {
                name: {
                    "node_id": joint.node_id,
                    "position_min": joint.position_min,
                    "position_max": joint.position_max,
                    "max_velocity": joint.max_velocity,
                    "max_acceleration": joint.max_acceleration,
                }
                for name, joint in self.joints.items()
            },
        }
        serialized = json.dumps(
            canonical, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
        return hashlib.sha256(serialized).hexdigest()


def real_xacro_command_arguments(
    robot_xacro: str,
    profile: RealArmProfile,
    *,
    remote_heartbeat_topic: str = "/arm_remote_control/heartbeat",
) -> list[str]:
    """Build the identical real-robot xacro command on the board and PC."""

    if not re.fullmatch(
        r"/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*", remote_heartbeat_topic
    ):
        raise ValueError("remote_heartbeat_topic must be a safe absolute ROS topic")

    return [
        "xacro ",
        robot_xacro,
        " hardware_type:=real",
        " can_interface:=",
        profile.can_interface,
        " baudrate:=",
        str(profile.baudrate),
        " encoder_bw:=",
        str(profile.encoder_bw),
        " reduction_ratio:=",
        str(profile.reduction_ratio),
        " node_ids:=",
        profile.node_ids_csv,
        " joint_indices:=",
        profile.joint_indices_csv,
        " position_min:=",
        profile.position_min_csv,
        " position_max:=",
        profile.position_max_csv,
        " sync_period_us:=",
        str(profile.sync_period_us),
        " read_rate_hz:=",
        str(profile.read_rate_hz),
        " state_poll_rate_hz:=",
        str(profile.state_poll_rate_hz),
        " manual_feedback_timeout_ms:=",
        str(profile.manual_feedback_timeout_ms),
        " feedback_stale_timeout_ms:=",
        str(profile.feedback_stale_timeout_ms),
        " enable_emcy_monitor:=",
        str(profile.enable_emcy_monitor).lower(),
        " max_command_step_rad:=",
        str(profile.max_command_step_rad),
        " max_following_error_rad:=",
        str(profile.max_following_error_rad),
        " following_error_cycles:=",
        str(profile.following_error_cycles),
        " remote_heartbeat_topic:=",
        remote_heartbeat_topic,
        " remote_profile_fingerprint:=",
        profile.fingerprint,
        " remote_watchdog_timeout_ms:=",
        str(profile.remote_hard_timeout_ms),
        " remote_stop_velocity_threshold_rad_s:=",
        str(profile.remote_stop_velocity_threshold_rad_s),
        " require_full_arm:=true",
        " require_position_limits:=true",
    ]


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return value


def _bool(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{label} must be true or false")
    return value


def _text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(
        r"[A-Za-z0-9_.:-]+", value.strip()
    ):
        raise ValueError(f"{label} contains unsupported characters")
    return value.strip()


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _integer(value: Any, label: str) -> int:
    number = _number(value, label)
    if not number.is_integer():
        raise ValueError(f"{label} must be an integer")
    return int(number)


def _positive(value: Any, label: str) -> float:
    number = _number(value, label)
    if number <= 0.0:
        raise ValueError(f"{label} must be greater than zero")
    return number


def _nonnegative(value: Any, label: str) -> float:
    number = _number(value, label)
    if number < 0.0:
        raise ValueError(f"{label} must be non-negative")
    return number


def _position(
    value: Any, label: str, *, allow_missing: bool
) -> Optional[float]:
    if value is None and allow_missing:
        return None
    return _number(value, label)


def load_real_arm_profile(
    path: str, *, require_calibrated: bool = True
) -> RealArmProfile:
    """Load a profile; real MoveIt callers must keep require_calibrated=True."""

    profile_path = Path(path)
    try:
        raw = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot load real-arm profile {profile_path}: {exc}") from exc

    root = _mapping(raw, "profile")
    if set(root) != {"schema_version", "calibrated", "hardware", "joints"}:
        raise ValueError(
            "profile must contain exactly schema_version, calibrated, hardware, and joints"
        )
    if _integer(root["schema_version"], "schema_version") != 1:
        raise ValueError("unsupported schema_version; expected 1")

    calibrated = _bool(root["calibrated"], "calibrated")
    if require_calibrated and not calibrated:
        raise ValueError(
            "real-arm profile is not calibrated; measure all six position limits "
            "and set calibrated: true only after review"
        )

    hardware = _mapping(root["hardware"], "hardware")
    expected_hardware = {
        "can_interface",
        "baudrate",
        "encoder_bw",
        "reduction_ratio",
        "sync_period_us",
        "controller_update_rate_hz",
        "read_rate_hz",
        "state_poll_rate_hz",
        "manual_feedback_timeout_ms",
        "feedback_stale_timeout_ms",
        "enable_emcy_monitor",
        "max_command_step_rad",
        "max_following_error_rad",
        "following_error_cycles",
        "remote_heartbeat_period_ms",
        "remote_soft_timeout_ms",
        "remote_hard_timeout_ms",
        "remote_stop_velocity_threshold_rad_s",
    }
    if set(hardware) != expected_hardware:
        missing = sorted(expected_hardware - set(hardware))
        extra = sorted(set(hardware) - expected_hardware)
        raise ValueError(f"hardware keys mismatch; missing={missing}, extra={extra}")

    joints_raw = _mapping(root["joints"], "joints")
    if set(joints_raw) != set(JOINT_NAMES):
        raise ValueError(f"joints must contain exactly {', '.join(JOINT_NAMES)}")

    allow_missing = not require_calibrated and not calibrated
    parsed_joints: Dict[str, RealJointConfig] = {}
    node_ids = set()
    for name in JOINT_NAMES:
        raw_joint = _mapping(joints_raw[name], f"joints.{name}")
        expected_joint = {
            "node_id",
            "position_min",
            "position_max",
            "max_velocity",
            "max_acceleration",
        }
        if set(raw_joint) != expected_joint:
            raise ValueError(f"joints.{name} has unexpected or missing keys")

        node_id = _integer(raw_joint["node_id"], f"joints.{name}.node_id")
        if node_id < 1 or node_id > 127:
            raise ValueError(f"joints.{name}.node_id must be in [1, 127]")
        if node_id in node_ids:
            raise ValueError(f"duplicate CANopen node_id {node_id}")
        node_ids.add(node_id)

        position_min = _position(
            raw_joint["position_min"],
            f"joints.{name}.position_min",
            allow_missing=allow_missing,
        )
        position_max = _position(
            raw_joint["position_max"],
            f"joints.{name}.position_max",
            allow_missing=allow_missing,
        )
        if position_min is None or position_max is None:
            # Only an explicitly uncalibrated profile may reach this branch.
            position_min, position_max = URDF_POSITION_LIMITS[name]
        elif position_min >= position_max:
            raise ValueError(f"joints.{name} position_min must be less than position_max")

        urdf_min, urdf_max = URDF_POSITION_LIMITS[name]
        if position_min < urdf_min or position_max > urdf_max:
            raise ValueError(
                f"joints.{name} position limits [{position_min}, {position_max}] "
                f"exceed URDF limits [{urdf_min}, {urdf_max}]"
            )

        parsed_joints[name] = RealJointConfig(
            node_id=node_id,
            position_min=position_min,
            position_max=position_max,
            max_velocity=_positive(
                raw_joint["max_velocity"], f"joints.{name}.max_velocity"
            ),
            max_acceleration=_positive(
                raw_joint["max_acceleration"], f"joints.{name}.max_acceleration"
            ),
        )

    controller_rate = _integer(
        hardware["controller_update_rate_hz"], "hardware.controller_update_rate_hz"
    )
    if controller_rate <= 0:
        raise ValueError("hardware.controller_update_rate_hz must be greater than zero")
    baudrate = _integer(hardware["baudrate"], "hardware.baudrate")
    encoder_bw = _integer(hardware["encoder_bw"], "hardware.encoder_bw")
    sync_period_us = _integer(hardware["sync_period_us"], "hardware.sync_period_us")
    manual_timeout = _integer(
        hardware["manual_feedback_timeout_ms"], "hardware.manual_feedback_timeout_ms"
    )
    stale_timeout = _integer(
        hardware["feedback_stale_timeout_ms"], "hardware.feedback_stale_timeout_ms"
    )
    following_error_cycles = _integer(
        hardware["following_error_cycles"], "hardware.following_error_cycles"
    )
    if baudrate < 0 or encoder_bw <= 0 or encoder_bw > 31 or sync_period_us < 0:
        raise ValueError(
            "baudrate/sync_period_us must be non-negative and encoder_bw in [1, 31]"
        )
    if sync_period_us != 0:
        raise ValueError(
            "hardware.sync_period_us must be 0 for the fail-closed real-arm "
            "MoveIt path; timer SYNC is not certified for six-axis target transactions"
        )
    if manual_timeout <= 0 or stale_timeout <= manual_timeout:
        raise ValueError(
            "manual_feedback_timeout_ms must be positive and "
            "feedback_stale_timeout_ms must be greater than it"
        )
    if not _bool(hardware["enable_emcy_monitor"], "hardware.enable_emcy_monitor"):
        raise ValueError("hardware.enable_emcy_monitor must be true for real MoveIt")
    max_command_step = _positive(
        hardware["max_command_step_rad"], "hardware.max_command_step_rad"
    )
    max_following_error = _positive(
        hardware["max_following_error_rad"], "hardware.max_following_error_rad"
    )
    if following_error_cycles <= 0:
        raise ValueError("hardware.following_error_cycles must be greater than zero")

    remote_heartbeat_period_ms = _integer(
        hardware["remote_heartbeat_period_ms"],
        "hardware.remote_heartbeat_period_ms",
    )
    remote_soft_timeout_ms = _integer(
        hardware["remote_soft_timeout_ms"], "hardware.remote_soft_timeout_ms"
    )
    remote_hard_timeout_ms = _integer(
        hardware["remote_hard_timeout_ms"], "hardware.remote_hard_timeout_ms"
    )
    if remote_heartbeat_period_ms <= 0:
        raise ValueError(
            "hardware.remote_heartbeat_period_ms must be greater than zero"
        )
    if remote_soft_timeout_ms <= remote_heartbeat_period_ms:
        raise ValueError(
            "hardware.remote_soft_timeout_ms must be greater than the heartbeat period"
        )
    if remote_hard_timeout_ms <= remote_soft_timeout_ms:
        raise ValueError(
            "hardware.remote_hard_timeout_ms must be greater than the soft timeout"
        )

    return RealArmProfile(
        calibrated=calibrated,
        can_interface=_text(hardware["can_interface"], "hardware.can_interface"),
        baudrate=baudrate,
        encoder_bw=encoder_bw,
        reduction_ratio=_positive(
            hardware["reduction_ratio"], "hardware.reduction_ratio"
        ),
        sync_period_us=sync_period_us,
        controller_update_rate_hz=controller_rate,
        read_rate_hz=_positive(hardware["read_rate_hz"], "hardware.read_rate_hz"),
        state_poll_rate_hz=_nonnegative(
            hardware["state_poll_rate_hz"], "hardware.state_poll_rate_hz"
        ),
        manual_feedback_timeout_ms=manual_timeout,
        feedback_stale_timeout_ms=stale_timeout,
        enable_emcy_monitor=_bool(
            hardware["enable_emcy_monitor"], "hardware.enable_emcy_monitor"
        ),
        max_command_step_rad=max_command_step,
        max_following_error_rad=max_following_error,
        following_error_cycles=following_error_cycles,
        remote_heartbeat_period_ms=remote_heartbeat_period_ms,
        remote_soft_timeout_ms=remote_soft_timeout_ms,
        remote_hard_timeout_ms=remote_hard_timeout_ms,
        remote_stop_velocity_threshold_rad_s=_positive(
            hardware["remote_stop_velocity_threshold_rad_s"],
            "hardware.remote_stop_velocity_threshold_rad_s",
        ),
        joints=parsed_joints,
    )
