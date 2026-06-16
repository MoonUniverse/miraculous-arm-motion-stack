#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from arm_isaac_common import DEFAULT_BACKEND_CONFIG, DEFAULT_DRIVE_CONFIG, load_backend_config, load_yaml

from isaaclab.app import AppLauncher


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Patch ARM Isaac USD joint drive parameters.")
    parser.add_argument("--backend_config", default=str(DEFAULT_BACKEND_CONFIG))
    parser.add_argument("--drive_config", default=str(DEFAULT_DRIVE_CONFIG))
    parser.add_argument("--usd_path", default="", help="Override the USD path from backend config.")
    parser.add_argument("--dry_run", action="store_true", help="Print planned changes without writing the USD.")
    parser.add_argument("--force_exit", action="store_true")
    AppLauncher.add_app_launcher_args(parser)
    return parser.parse_args()


args_cli = parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics  # noqa: E402


def find_joint_prim(stage: Usd.Stage, joint_name: str) -> Usd.Prim:
    matches = [prim for prim in stage.Traverse() if prim.GetName() == joint_name]
    matches.extend(
        prim for prim in stage.Traverse() if str(prim.GetPath()).endswith(f"/{joint_name}") and prim not in matches
    )

    for prim in matches:
        if prim.GetTypeName() in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint", "PhysicsJoint"):
            return prim
    for prim in matches:
        if "Joint" in str(prim.GetTypeName()):
            return prim

    return Usd.Prim()


def remove_stale_drive_from_non_joint(stage: Usd.Stage, joint_name: str, selected_prim: Usd.Prim) -> None:
    for prim in stage.Traverse():
        if prim.GetName() != joint_name or prim == selected_prim:
            continue
        if "Joint" in str(prim.GetTypeName()):
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if drive:
            prim.RemoveAPI(UsdPhysics.DriveAPI, "angular")
            print(f"DRIVE_CLEANUP joint={joint_name} prim={prim.GetPath()}", flush=True)


def patch_drive(prim: Usd.Prim, values: dict[str, object]) -> None:
    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateStiffnessAttr().Set(float(values["stiffness"]))
    drive.CreateDampingAttr().Set(float(values["damping"]))
    drive.CreateMaxForceAttr().Set(float(values["max_force"]))

    target_type = str(values.get("target_type", "position"))
    if target_type == "position":
        drive.CreateTargetPositionAttr().Set(0.0)
    elif target_type == "velocity":
        drive.CreateTargetVelocityAttr().Set(0.0)
    elif target_type != "none":
        raise ValueError(f"Unsupported target_type '{target_type}' for {prim.GetPath()}")


def main() -> int:
    backend_config = load_backend_config(args_cli.backend_config)
    drive_config = load_yaml(args_cli.drive_config)
    usd_path = Path(args_cli.usd_path or backend_config["usd_path"]).resolve()
    if not usd_path.exists():
        print(f"USD_MISSING {usd_path}", flush=True)
        return 2

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        print(f"USD_OPEN_FAIL {usd_path}", flush=True)
        return 3

    ok = True
    for joint_name, values in drive_config.get("joint_drives", {}).items():
        prim = find_joint_prim(stage, joint_name)
        if not prim or not prim.IsValid():
            print(f"DRIVE_PATCH_FAIL joint={joint_name} reason=missing_prim", flush=True)
            ok = False
            continue
        print(
            "DRIVE_PATCH "
            f"joint={joint_name} prim={prim.GetPath()} "
            f"stiffness={values['stiffness']} damping={values['damping']} "
            f"max_force={values['max_force']} target_type={values.get('target_type', 'position')}",
            flush=True,
        )
        if not args_cli.dry_run:
            remove_stale_drive_from_non_joint(stage, joint_name, prim)
            patch_drive(prim, values)

    if not ok:
        return 4
    if not args_cli.dry_run:
        stage.GetRootLayer().Save()
        print(f"USD_SAVED {usd_path}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        try:
            exit_code = main()
        except SystemExit as exc:
            print(f"RUNTIME_ABORT SystemExit code={exc.code}", flush=True)
            exit_code = int(exc.code) if isinstance(exc.code, int) and exc.code else 1
    finally:
        if args_cli.force_exit:
            print("APP_CLOSE SKIP force_exit=on", flush=True)
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(exit_code)
        simulation_app.close()
    raise SystemExit(exit_code)
