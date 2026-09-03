#!/usr/bin/env python3
"""On-robot relay: joints -> TCP + namespaced /r1lite/joint_states.

Sources (merged):
  1) /joint_states (vendor, if publishing)
  2) /hdas/feedback_arm_{left,right} + /hdas/feedback_gripper_{left,right}

Naming:
  ROS node : /r1lite/joint_tcp_relay
  ROS topic: /r1lite/joint_states
  TCP port : 8765
"""

from __future__ import annotations

import argparse
import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

R1LITE_NS = "r1lite"
R1LITE_NODE = "joint_tcp_relay"
R1LITE_JOINT_TOPIC = "/r1lite/joint_states"
VENDOR_JOINT_TOPIC = "/joint_states"

LEFT_ARM = [f"left_arm_joint{i}" for i in range(1, 7)]
RIGHT_ARM = [f"right_arm_joint{i}" for i in range(1, 7)]
LEFT_FINGERS = ("left_gripper_finger_joint1", "left_gripper_finger_joint2")
RIGHT_FINGERS = ("right_gripper_finger_joint1", "right_gripper_finger_joint2")
# HDAS /hdas/feedback_gripper_* publishes a single scalar (~0 closed … ~100 open),
# while /joint_states finger joints are meters in [0, 0.05] / [-0.05, 0].
# Mixing raw HDAS into finger names caused TCP values to jump ~0 ↔ ~100.
HDAS_GRIPPER_OPEN = 100.0
FINGER_TRAVEL_M = 0.05
VENDOR_SKIP_FINGERS = set(LEFT_FINGERS + RIGHT_FINGERS)


def hdas_gripper_to_fingers(g: float) -> tuple[float, float]:
    """Map HDAS gripper scalar to (finger1, finger2) prismatic meters."""
    open_frac = max(0.0, min(1.0, float(g) / HDAS_GRIPPER_OPEN))
    return open_frac * FINGER_TRAVEL_M, -open_frac * FINGER_TRAVEL_M


FULL_ORDER = [
    "steer_motor_joint1",
    "wheel_motor_joint1",
    "steer_motor_joint2",
    "wheel_motor_joint2",
    "steer_motor_joint3",
    "wheel_motor_joint3",
    "torso_joint1",
    "torso_joint2",
    "torso_joint3",
    *LEFT_ARM,
    "left_gripper_finger_joint1",
    "left_gripper_finger_joint2",
    *RIGHT_ARM,
    "right_gripper_finger_joint1",
    "right_gripper_finger_joint2",
]


class Relay(Node):
    def __init__(self, vendor_topic: str, publish_topic: str) -> None:
        super().__init__(R1LITE_NODE, namespace=R1LITE_NS)
        self._lock = threading.Lock()
        self._q: dict[str, float] = {n: 0.0 for n in FULL_ORDER}
        self._have_data = False
        self._pub = self.create_publisher(JointState, publish_topic, 10)
        qos_tl = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(JointState, vendor_topic, self._on_vendor, qos_tl)
        self.create_subscription(JointState, "/hdas/feedback_arm_left", self._on_left, qos_tl)
        self.create_subscription(JointState, "/hdas/feedback_arm_right", self._on_right, qos_tl)
        self.create_subscription(JointState, "/hdas/feedback_gripper_left", self._on_g_left, qos_tl)
        self.create_subscription(JointState, "/hdas/feedback_gripper_right", self._on_g_right, qos_tl)
        self.get_logger().info(
            f"node=/{R1LITE_NS}/{R1LITE_NODE} sub={vendor_topic}+hdas/feedback_* pub={publish_topic}"
        )

    def _merge(self, mapping: dict[str, float]) -> None:
        with self._lock:
            self._q.update(mapping)
            self._have_data = True
            payload_names = list(FULL_ORDER)
            payload_pos = [float(self._q.get(n, 0.0)) for n in payload_names]
        out = JointState()
        out.name = payload_names
        out.position = payload_pos
        self._pub.publish(out)

    def _on_vendor(self, msg: JointState) -> None:
        # Prefer HDAS gripper conversion for fingers; jointTracker often reports 0.
        m = {
            str(n): float(p)
            for n, p in zip(msg.name, msg.position)
            if str(n) not in VENDOR_SKIP_FINGERS
        }
        self._merge(m)

    def _on_left(self, msg: JointState) -> None:
        m = {}
        if len(msg.position) >= 6:
            for i, n in enumerate(LEFT_ARM):
                m[n] = float(msg.position[i])
        for n, p in zip(msg.name, msg.position):
            if str(n) not in VENDOR_SKIP_FINGERS:
                m[str(n)] = float(p)
        self._merge(m)

    def _on_right(self, msg: JointState) -> None:
        m = {}
        if len(msg.position) >= 6:
            for i, n in enumerate(RIGHT_ARM):
                m[n] = float(msg.position[i])
        for n, p in zip(msg.name, msg.position):
            if str(n) not in VENDOR_SKIP_FINGERS:
                m[str(n)] = float(p)
        self._merge(m)

    def _on_g_left(self, msg: JointState) -> None:
        m = {}
        if len(msg.position) >= 2:
            # Already finger-like (meters); keep as-is.
            m[LEFT_FINGERS[0]] = float(msg.position[0])
            m[LEFT_FINGERS[1]] = float(msg.position[1])
        elif len(msg.position) == 1:
            f1, f2 = hdas_gripper_to_fingers(float(msg.position[0]))
            m[LEFT_FINGERS[0]] = f1
            m[LEFT_FINGERS[1]] = f2
        self._merge(m)

    def _on_g_right(self, msg: JointState) -> None:
        m = {}
        if len(msg.position) >= 2:
            m[RIGHT_FINGERS[0]] = float(msg.position[0])
            m[RIGHT_FINGERS[1]] = float(msg.position[1])
        elif len(msg.position) == 1:
            f1, f2 = hdas_gripper_to_fingers(float(msg.position[0]))
            m[RIGHT_FINGERS[0]] = f1
            m[RIGHT_FINGERS[1]] = f2
        self._merge(m)

    def snapshot_line(self) -> str | None:
        with self._lock:
            if not self._have_data:
                return None
            payload = {
                "name": list(FULL_ORDER),
                "position": [float(self._q.get(n, 0.0)) for n in FULL_ORDER],
            }
            return json.dumps(payload, separators=(",", ":")) + "\n"


def serve(relay: Relay, host: str, port: int, hz: float) -> None:
    period = 1.0 / hz if hz > 0 else 0.05
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(8)
    print(f"TCP joint relay listening on {host}:{port} @ {hz} Hz", flush=True)

    def client_loop(conn: socket.socket, addr) -> None:
        print(f"client connected {addr}", flush=True)
        try:
            with conn:
                while True:
                    line = relay.snapshot_line()
                    if line:
                        conn.sendall(line.encode("utf-8"))
                    time.sleep(period)
        except Exception as exc:
            print(f"client {addr} closed: {exc!r}", flush=True)

    while True:
        conn, addr = srv.accept()
        threading.Thread(target=client_loop, args=(conn, addr), daemon=True).start()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vendor-topic", default=VENDOR_JOINT_TOPIC)
    parser.add_argument("--publish-topic", default=R1LITE_JOINT_TOPIC)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--hz", type=float, default=50.0)
    args = parser.parse_args()

    rclpy.init()
    relay = Relay(args.vendor_topic, args.publish_topic)
    threading.Thread(target=rclpy.spin, args=(relay,), daemon=True).start()
    # Serve immediately; clients tolerate empty until first sample.
    threading.Thread(
        target=lambda: (
            time.sleep(5.0),
            print(
                "WARN: still no joint data after 5s"
                if relay.snapshot_line() is None
                else f"joint stream live: {relay.snapshot_line()[:80]}...",
                flush=True,
            ),
        ),
        daemon=True,
    ).start()
    try:
        serve(relay, args.host, args.port, args.hz)
    finally:
        relay.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
