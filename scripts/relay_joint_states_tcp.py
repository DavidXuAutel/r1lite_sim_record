#!/usr/bin/env python3
"""On-robot relay: vendor /joint_states -> TCP + namespaced /r1lite/joint_states.

Naming rules (do not collide with Franka on 10.229.20.125):
  ROS node : /r1lite/joint_tcp_relay
  ROS topic: /r1lite/joint_states   (published)
  TCP port : 8765

Franka subscriptions are left untouched. This node only *reads* the vendor
topic /joint_states on the robot and republishes under /r1lite/*.
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


class Relay(Node):
    def __init__(self, vendor_topic: str, publish_topic: str) -> None:
        # Explicit namespaced node: /r1lite/joint_tcp_relay
        super().__init__(R1LITE_NODE, namespace=R1LITE_NS)
        self._lock = threading.Lock()
        self.latest: dict | None = None
        self._pub = self.create_publisher(JointState, publish_topic, 10)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(JointState, vendor_topic, self._cb, qos)
        self.get_logger().info(
            f"node=/{R1LITE_NS}/{R1LITE_NODE}  sub={vendor_topic}  pub={publish_topic}"
        )

    def _cb(self, msg: JointState) -> None:
        # Republish under /r1lite/joint_states (never publish to bare /joint_states).
        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)
        out.position = list(msg.position)
        out.velocity = list(msg.velocity)
        out.effort = list(msg.effort)
        self._pub.publish(out)

        payload = {
            "name": list(msg.name),
            "position": [float(x) for x in msg.position],
            "stamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            },
        }
        with self._lock:
            self.latest = payload

    def snapshot_line(self) -> str | None:
        with self._lock:
            if self.latest is None:
                return None
            return json.dumps(self.latest, separators=(",", ":")) + "\n"


def serve(relay: Relay, host: str, port: int, hz: float) -> None:
    period = 1.0 / hz if hz > 0 else 0.05
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(4)
    print(
        f"TCP joint relay listening on {host}:{port} @ {hz} Hz "
        f"(ROS pub={R1LITE_JOINT_TOPIC})",
        flush=True,
    )

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
    parser.add_argument(
        "--vendor-topic",
        default=VENDOR_JOINT_TOPIC,
        help="Robot-side vendor topic to read (default /joint_states)",
    )
    parser.add_argument(
        "--publish-topic",
        default=R1LITE_JOINT_TOPIC,
        help="Namespaced topic to publish (default /r1lite/joint_states)",
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--hz", type=float, default=50.0)
    args = parser.parse_args()

    rclpy.init()
    relay = Relay(args.vendor_topic, args.publish_topic)
    threading.Thread(target=rclpy.spin, args=(relay,), daemon=True).start()
    t0 = time.time()
    while relay.snapshot_line() is None and time.time() - t0 < 10:
        time.sleep(0.1)
    if relay.snapshot_line() is None:
        print(f"WARN: no data on {args.vendor_topic} yet; still listening", flush=True)
    try:
        serve(relay, args.host, args.port, args.hz)
    finally:
        relay.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
