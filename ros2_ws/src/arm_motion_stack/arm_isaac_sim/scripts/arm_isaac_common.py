from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
STACK_DIR = PACKAGE_DIR.parent
DEFAULT_BACKEND_CONFIG = PACKAGE_DIR / "config" / "arm_isaac_backend.yaml"
DEFAULT_DRIVE_CONFIG = PACKAGE_DIR / "config" / "arm_isaac_drives.yaml"


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def resolve_stack_path(value: str | Path, base_dir: str | Path = PACKAGE_DIR) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    return (Path(base_dir) / path).resolve()


def load_backend_config(path: str | Path = DEFAULT_BACKEND_CONFIG) -> dict[str, Any]:
    config_path = Path(path).resolve()
    config = load_yaml(config_path)
    config["config_path"] = str(config_path)
    config["config_dir"] = str(config_path.parent)
    config["usd_path"] = str(resolve_stack_path(config["usd_path"], config_path.parent))
    return config


def joint_names_from_config(config: dict[str, Any]) -> tuple[str, ...]:
    names = config.get("joint_names", ())
    if not isinstance(names, list) or not all(isinstance(name, str) for name in names):
        raise ValueError("joint_names must be a list of strings")
    return tuple(names)


def initial_joint_positions(config: dict[str, Any], joint_names: tuple[str, ...]) -> dict[str, float]:
    values = config.get("initial_joint_positions", {})
    if not isinstance(values, dict):
        raise ValueError("initial_joint_positions must be a mapping")
    return {name: float(values.get(name, 0.0)) for name in joint_names}
