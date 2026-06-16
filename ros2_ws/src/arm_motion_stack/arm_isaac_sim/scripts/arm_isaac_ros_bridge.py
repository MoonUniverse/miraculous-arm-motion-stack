#!/usr/bin/env python3
from __future__ import annotations

import json
import select
import socket
from pathlib import Path
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
import yaml


def load_config(path: str) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


class ArmIsaacRosBridge(Node):
    def __init__(self) -> None:
        super().__init__("arm_isaac_ros_bridge")
        default_config = str(
            Path(get_package_share_directory("arm_isaac_sim")) / "config" / "arm_isaac_backend.yaml"
        )
        self.declare_parameter("backend_config", default_config)
        config_path = self.get_parameter("backend_config").get_parameter_value().string_value
        self.config = load_config(config_path)
        self.joint_names = list(self.config["joint_names"])
        udp_config = self.config.get("udp", {})
        self.host = str(udp_config.get("host", "127.0.0.1"))
        self.command_addr = (self.host, int(udp_config.get("command_port", 55100)))
        self.state_addr = (self.host, int(udp_config.get("state_port", 55101)))

        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_socket.setblocking(False)
        self.state_socket.bind(self.state_addr)

        self.state_pub = self.create_publisher(JointState, str(self.config["joint_states_topic"]), 10)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.command_sub = self.create_subscription(
            JointState,
            str(self.config["joint_commands_topic"]),
            self.command_callback,
            10,
        )
        self.timer = self.create_timer(0.002, self.poll_state_socket)
        self.get_logger().info(
            "ARM Isaac ROS bridge ready: "
            f"commands={self.config['joint_commands_topic']}->{self.command_addr} "
            f"states={self.state_addr}->{self.config['joint_states_topic']}"
        )

    def command_callback(self, msg: JointState) -> None:
        payload = {
            "name": list(msg.name),
            "position": list(msg.position),
        }
        self.command_socket.sendto(json.dumps(payload).encode("utf-8"), self.command_addr)

    def poll_state_socket(self) -> None:
        while True:
            readable, _, _ = select.select([self.state_socket], [], [], 0.0)
            if not readable:
                return
            data, _ = self.state_socket.recvfrom(65536)
            try:
                payload = json.loads(data.decode("utf-8"))
            except json.JSONDecodeError as exc:
                self.get_logger().warn(f"Ignoring invalid Isaac state datagram: {exc}")
                continue

            sim_time = float(payload.get("time", 0.0))
            msg = JointState()
            msg.header.stamp.sec = int(sim_time)
            msg.header.stamp.nanosec = int((sim_time - int(sim_time)) * 1e9)
            msg.name = list(payload.get("name", self.joint_names))
            msg.position = [float(value) for value in payload.get("position", [])]
            msg.velocity = [float(value) for value in payload.get("velocity", [])]
            msg.effort = [0.0 for _ in msg.name]
            self.state_pub.publish(msg)

            clock = Clock()
            clock.clock = msg.header.stamp
            self.clock_pub.publish(clock)


def main() -> None:
    rclpy.init()
    node = ArmIsaacRosBridge()
    try:
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except Exception as exc:
            if "context is not valid" not in str(exc):
                raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
