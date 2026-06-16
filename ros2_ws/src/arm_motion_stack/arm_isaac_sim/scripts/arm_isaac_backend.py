#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import select
import socket
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from arm_isaac_common import DEFAULT_BACKEND_CONFIG, initial_joint_positions, joint_names_from_config, load_backend_config

from isaaclab.app import AppLauncher


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the ARM IsaacLab ROS 2 joint bridge backend.")
    parser.add_argument("--backend_config", default=str(DEFAULT_BACKEND_CONFIG))
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run; 0 means until stopped.")
    parser.add_argument("--force_exit", action="store_true")
    AppLauncher.add_app_launcher_args(parser)
    return parser.parse_args()


args_cli = parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg, AssetBaseCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402


CONFIG = load_backend_config(args_cli.backend_config)
JOINT_NAMES = joint_names_from_config(CONFIG)
USD_PATH = Path(CONFIG["usd_path"])


def build_robot_cfg() -> ArticulationCfg:
    articulation = CONFIG.get("articulation", {})
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
            joint_pos=initial_joint_positions(CONFIG, JOINT_NAMES),
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
class ArmBackendSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )
    robot = build_robot_cfg().replace(prim_path="{ENV_REGEX_NS}/Robot")


class IsaacUdpBridge:
    def __init__(self, robot: Articulation, joint_ids: list[int]):
        self.robot = robot
        self.joint_ids = joint_ids
        self.command_timeout_sec = float(CONFIG.get("command_timeout_sec", 1.0))
        self.targets = robot.data.default_joint_pos[:, joint_ids].clone()
        self.last_command_time = 0.0
        udp_config = CONFIG.get("udp", {})
        self.host = str(udp_config.get("host", "127.0.0.1"))
        self.command_addr = (self.host, int(udp_config.get("command_port", 55100)))
        self.state_addr = (self.host, int(udp_config.get("state_port", 55101)))
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket.setblocking(False)
        self.command_socket.bind(self.command_addr)
        self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def close(self) -> None:
        self.command_socket.close()
        self.state_socket.close()

    def poll_commands(self, sim_time: float) -> None:
        while True:
            readable, _, _ = select.select([self.command_socket], [], [], 0.0)
            if not readable:
                return
            data, _ = self.command_socket.recvfrom(65536)
            try:
                payload = json.loads(data.decode("utf-8"))
            except json.JSONDecodeError:
                continue
            self.command_callback(payload, sim_time)

    def command_callback(self, payload: dict[str, object], sim_time: float) -> None:
        names = payload.get("name", [])
        positions = payload.get("position", [])
        if not isinstance(names, list) or not isinstance(positions, list):
            return
        target = self.targets.clone()
        name_to_position = {name: positions[index] for index, name in enumerate(names) if index < len(positions)}
        for index, joint_name in enumerate(JOINT_NAMES):
            if joint_name in name_to_position:
                target[:, index] = float(name_to_position[joint_name])
        self.targets = target
        self.last_command_time = sim_time

    def apply_command(self, sim_time: float) -> None:
        age = sim_time - self.last_command_time
        if self.command_timeout_sec > 0.0 and age > self.command_timeout_sec:
            self.targets = self.robot.data.joint_pos[:, self.joint_ids].clone()
            self.last_command_time = sim_time
        self.robot.set_joint_position_target(self.targets, joint_ids=self.joint_ids)

    def publish_state(self, sim_time_sec: float) -> None:
        payload = {
            "time": sim_time_sec,
            "name": list(JOINT_NAMES),
            "position": self.robot.data.joint_pos[0, self.joint_ids].detach().cpu().tolist(),
            "velocity": self.robot.data.joint_vel[0, self.joint_ids].detach().cpu().tolist(),
        }
        self.state_socket.sendto(json.dumps(payload).encode("utf-8"), self.state_addr)


def main() -> int:
    if not USD_PATH.exists():
        print(f"USD_MISSING {USD_PATH}", flush=True)
        return 2

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(device=args_cli.device))
    sim.set_camera_view([1.2, -1.4, 1.0], [0.0, 0.0, 0.2])
    scene = InteractiveScene(ArmBackendSceneCfg(num_envs=int(CONFIG.get("num_envs", 1)), env_spacing=2.0))
    sim.reset()
    robot: Articulation = scene["robot"]
    joint_ids = [robot.data.joint_names.index(name) for name in JOINT_NAMES]
    bridge = IsaacUdpBridge(robot, joint_ids)

    publish_rate_hz = float(CONFIG.get("publish_rate_hz", 100.0))
    publish_period = 1.0 / publish_rate_hz if publish_rate_hz > 0.0 else sim.get_physics_dt()
    next_publish_time = 0.0
    start_time = sim.current_time

    print(
        f"ARM_ISAAC_BACKEND_READY usd={USD_PATH} commands={CONFIG['joint_commands_topic']} "
        f"states={CONFIG['joint_states_topic']} udp_command={bridge.command_addr} "
        f"udp_state={bridge.state_addr} joints={','.join(JOINT_NAMES)}",
        flush=True,
    )

    try:
        while simulation_app.is_running():
            bridge.poll_commands(sim.current_time)
            bridge.apply_command(sim.current_time)
            scene.write_data_to_sim()
            sim.step()
            scene.update(sim.get_physics_dt())
            if sim.current_time >= next_publish_time:
                bridge.publish_state(sim.current_time)
                next_publish_time = sim.current_time + publish_period
            if args_cli.duration > 0.0 and sim.current_time - start_time >= args_cli.duration:
                break
    finally:
        bridge.close()

    print("ARM_ISAAC_BACKEND_STOPPED", flush=True)
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
