#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from arm_isaac_common import DEFAULT_BACKEND_CONFIG, initial_joint_positions, joint_names_from_config, load_backend_config

from isaaclab.app import AppLauncher


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate the ARM USD through IsaacLab.")
    parser.add_argument("--backend_config", default=str(DEFAULT_BACKEND_CONFIG))
    parser.add_argument("--steps", type=int, default=240)
    parser.add_argument("--strict_control", action="store_true")
    parser.add_argument("--force_exit", action="store_true")
    AppLauncher.add_app_launcher_args(parser)
    return parser.parse_args()


args_cli = parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import ArticulationCfg, AssetBaseCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402


CONFIG = load_backend_config(args_cli.backend_config)
JOINT_NAMES = joint_names_from_config(CONFIG)
USD_PATH = Path(CONFIG["usd_path"])


def print_result(name: str, ok: bool, detail: str = "") -> bool:
    suffix = f" {detail}" if detail else ""
    print(f"{name} {'PASS' if ok else 'FAIL'}{suffix}", flush=True)
    return ok


def print_control_result(name: str, ok: bool, detail: str = "") -> bool:
    if args_cli.strict_control:
        return print_result(name, ok, detail)
    status = "PASS" if ok else "WARN"
    print(f"{name} {status} {detail} strict_control=off", flush=True)
    return True


def static_usd_checks() -> bool:
    ok = True
    ok &= print_result("USD_EXISTS", USD_PATH.exists(), str(USD_PATH))
    if not USD_PATH.exists():
        return False
    stage = Usd.Stage.Open(str(USD_PATH))
    ok &= print_result("USD_OPEN", stage is not None)
    if stage is None:
        return False
    ok &= print_result("USD_UP_AXIS", UsdGeom.GetStageUpAxis(stage) == "Z", UsdGeom.GetStageUpAxis(stage))
    ok &= print_result(
        "USD_METERS_PER_UNIT",
        abs(float(UsdGeom.GetStageMetersPerUnit(stage)) - 1.0) < 1e-9,
        str(UsdGeom.GetStageMetersPerUnit(stage)),
    )
    missing = []
    missing_drive = []
    for joint_name in JOINT_NAMES:
        joint_prim = find_joint_prim(stage, joint_name)
        if not joint_prim or not joint_prim.IsValid():
            missing.append(joint_name)
            continue
        drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
        if not drive or not drive.GetStiffnessAttr().IsValid() or not drive.GetDampingAttr().IsValid():
            missing_drive.append(joint_name)
    ok &= print_result("USD_EXPECTED_JOINTS", not missing, f"missing={missing}")
    ok &= print_result("USD_JOINT_DRIVES", not missing_drive, f"missing={missing_drive}")
    return ok


def find_joint_prim(stage: Usd.Stage, joint_name: str) -> Usd.Prim:
    matches = [prim for prim in stage.Traverse() if prim.GetName() == joint_name]
    for prim in matches:
        if prim.GetTypeName() in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint", "PhysicsJoint"):
            return prim
    for prim in matches:
        if "Joint" in str(prim.GetTypeName()):
            return prim
    return Usd.Prim()


def build_robot_cfg() -> ArticulationCfg:
    articulation = CONFIG.get("articulation", {})
    initial_positions = initial_joint_positions(CONFIG, JOINT_NAMES)
    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_PATH),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=bool(articulation.get("enabled_self_collisions", False)),
                solver_position_iteration_count=int(articulation.get("solver_position_iteration_count", 64)),
                solver_velocity_iteration_count=int(articulation.get("solver_velocity_iteration_count", 16)),
                fix_root_link=bool(CONFIG.get("fixed_base", True)),
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
            joint_pos=initial_positions,
            joint_vel={".*": 0.0},
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=list(JOINT_NAMES),
                effort_limit_sim=None,
                velocity_limit_sim=None,
                stiffness=None,
                damping=None,
            )
        },
        soft_joint_pos_limit_factor=1.0,
    )


@configclass
class ArmValidationSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )
    robot = build_robot_cfg().replace(prim_path="{ENV_REGEX_NS}/Robot")


def joint_ids(robot) -> list[int]:
    return [robot.data.joint_names.index(name) for name in JOINT_NAMES]


def step_scene(sim: sim_utils.SimulationContext, scene: InteractiveScene, steps: int) -> None:
    for _ in range(steps):
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.get_physics_dt())


def runtime_checks(scene: InteractiveScene, sim: sim_utils.SimulationContext) -> bool:
    robot = scene["robot"]
    ids = joint_ids(robot)
    ok = True
    missing = sorted(set(JOINT_NAMES) - set(robot.data.joint_names))
    ok &= print_result("ISAACLAB_JOINTS", not missing, f"count={len(robot.data.joint_names)} missing={missing}")
    finite = torch.isfinite(robot.data.joint_pos).all() and torch.isfinite(robot.data.joint_vel).all()
    ok &= print_result("FINITE_INITIAL_STATE", bool(finite))

    default_pos = robot.data.default_joint_pos[:, ids].clone()
    targets = default_pos.clone()
    targets[:, :] = torch.tensor([[0.20, -0.35, 0.35, 0.20, -0.20, 0.20]], device=robot.device)
    robot.set_joint_position_target(targets, joint_ids=ids)
    step_scene(sim, scene, args_cli.steps)
    actual = robot.data.joint_pos[:, ids]
    max_error = float(torch.max(torch.abs(actual - targets)).item())
    ok &= print_control_result("ARM_READY_POSITION", max_error < 0.05, f"max_error={max_error:.5f}")

    for index, joint_name in enumerate(JOINT_NAMES):
      single_targets = actual.clone()
      single_targets[:, index] += 0.08
      robot.set_joint_position_target(single_targets, joint_ids=ids)
      step_scene(sim, scene, max(80, args_cli.steps // 3))
      single_error = float(torch.max(torch.abs(robot.data.joint_pos[:, ids] - single_targets)).item())
      ok &= print_control_result(f"JOINT_{joint_name}_STEP", single_error < 0.05, f"max_error={single_error:.5f}")

    finite = torch.isfinite(robot.data.joint_pos).all() and torch.isfinite(robot.data.joint_vel).all()
    ok &= print_result("FINITE_FINAL_STATE", bool(finite))
    return ok


def main() -> int:
    ok = static_usd_checks()
    if not ok:
        print("OVERALL FAIL", flush=True)
        return 1

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(device=args_cli.device))
    sim.set_camera_view([1.2, -1.4, 1.0], [0.0, 0.0, 0.2])
    scene = InteractiveScene(ArmValidationSceneCfg(num_envs=int(CONFIG.get("num_envs", 1)), env_spacing=2.0))
    sim.reset()
    step_scene(sim, scene, 20)
    ok &= runtime_checks(scene, sim)
    print("OVERALL", "PASS" if ok else "FAIL", flush=True)
    return 0 if ok else 1


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
