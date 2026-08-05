"""Shared, hardware-independent configuration for real-arm deployments."""

from .real_arm_profile import (
    JOINT_NAMES,
    RealArmProfile,
    RealJointConfig,
    load_real_arm_profile,
    real_xacro_command_arguments,
)

__all__ = [
    "JOINT_NAMES",
    "RealArmProfile",
    "RealJointConfig",
    "load_real_arm_profile",
    "real_xacro_command_arguments",
]
