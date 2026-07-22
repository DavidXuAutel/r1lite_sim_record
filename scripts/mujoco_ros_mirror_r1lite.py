#!/usr/bin/env python3
"""Mirror R1Lite joint states into a MuJoCo viewer.

Naming rules (isolated from Franka on 10.229.20.125):
  ROS node : /r1lite/mujoco_mirror
  ROS topic: /r1lite/joint_states   (optional; default off, use TCP)
  TCP      : 10.229.66.95:8765

Does NOT subscribe to bare /joint_states (Franka / twin keep that topic).
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import threading
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "glfw")
os.environ.setdefault("DISPLAY", ":1")

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402

R1LITE_NS = "r1lite"
R1LITE_NODE = "mujoco_mirror"
R1LITE_JOINT_TOPIC = "/r1lite/joint_states"

R1LITE_JOINT_NAMES = [
    "steer_motor_joint1",
    "wheel_motor_joint1",
    "steer_motor_joint2",
    "wheel_motor_joint2",
    "steer_motor_joint3",
    "wheel_motor_joint3",
    "torso_joint1",
    "torso_joint2",
    "torso_joint3",
    "left_arm_joint1",
    "left_arm_joint2",
    "left_arm_joint3",
    "left_arm_joint4",
    "left_arm_joint5",
    "left_arm_joint6",
    "left_gripper_finger_joint1",
    "left_gripper_finger_joint2",
    "right_arm_joint1",
    "right_arm_joint2",
    "right_arm_joint3",
    "right_arm_joint4",
    "right_arm_joint5",
    "right_arm_joint6",
    "right_gripper_finger_joint1",
    "right_gripper_finger_joint2",
]

DEFAULT_MODEL = "/home/yao/r1lite_mujoco_sync/r1lite.mujoco.urdf"
DEFAULT_TOPIC = R1LITE_JOINT_TOPIC
DEFAULT_TCP = ("10.229.66.95", 8765)


class JointStateStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.latest: dict[str, float] | None = None
        self.source = "none"

    def update(self, name_to_pos: dict[str, float], source: str) -> None:
        with self._lock:
            self.latest = name_to_pos
            self.source = source

    def snapshot(self) -> tuple[dict[str, float] | None, str]:
        with self._lock:
            return (None if self.latest is None else dict(self.latest), self.source)


def start_ros_subscriber(store: JointStateStore, topic: str) -> bool:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
    except Exception as exc:
        print(f"ROS unavailable: {exc!r}", flush=True)
        return False

    if topic.rstrip("/") == "/joint_states":
        print(
            "REFUSING to subscribe bare /joint_states (reserved for Franka/twin). "
            f"Use {R1LITE_JOINT_TOPIC}.",
            flush=True,
        )
        return False

    class Sub(Node):
        def __init__(self) -> None:
            super().__init__(R1LITE_NODE, namespace=R1LITE_NS)
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self.create_subscription(JointState, topic, self._cb, qos)

        def _cb(self, msg: JointState) -> None:
            names = list(msg.name)
            if not any(n.startswith("left_arm_joint") or n.startswith("torso_joint") for n in names):
                return
            mapping = {n: float(p) for n, p in zip(names, msg.position)}
            energy = sum(
                abs(mapping.get(k, 0.0))
                for k in (
                    "torso_joint1",
                    "torso_joint2",
                    "torso_joint3",
                    "left_arm_joint1",
                    "right_arm_joint1",
                )
            )
            if energy < 1e-3:
                return
            store.update(mapping, "ros")

    try:
        rclpy.init()
        node = Sub()
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        print(f"ROS subscriber /{R1LITE_NS}/{R1LITE_NODE} on {topic}", flush=True)
        return True
    except Exception as exc:
        print(f"ROS subscriber failed: {exc!r}", flush=True)
        return False


def start_tcp_client(store: JointStateStore, host: str, port: int) -> None:
    def loop() -> None:
        while True:
            try:
                with socket.create_connection((host, port), timeout=3.0) as sock:
                    sock_file = sock.makefile("r", encoding="utf-8", newline="\n")
                    print(f"TCP connected {host}:{port}", flush=True)
                    for line in sock_file:
                        line = line.strip()
                        if not line:
                            continue
                        payload = json.loads(line)
                        names = payload.get("name") or []
                        positions = payload.get("position") or []
                        store.update(
                            {str(n): float(p) for n, p in zip(names, positions)},
                            "tcp",
                        )
            except Exception as exc:
                print(f"TCP reconnect soon ({host}:{port}): {exc!r}", flush=True)
                time.sleep(1.0)

    threading.Thread(target=loop, daemon=True).start()


def main() -> int:
    parser = argparse.ArgumentParser(description="R1Lite MuJoCo ROS/TCP joint mirror")
    parser.add_argument("--model", default=os.environ.get("MUJOCO_MODEL", DEFAULT_MODEL))
    parser.add_argument("--topic", default=os.environ.get("MUJOCO_TOPIC", DEFAULT_TOPIC))
    parser.add_argument("--tcp-host", default=os.environ.get("R1LITE_TCP_HOST", DEFAULT_TCP[0]))
    parser.add_argument(
        "--tcp-port",
        type=int,
        default=int(os.environ.get("R1LITE_TCP_PORT", str(DEFAULT_TCP[1]))),
    )
    parser.add_argument(
        "--use-ros",
        action="store_true",
        help=f"Also subscribe {R1LITE_JOINT_TOPIC} (never bare /joint_states)",
    )
    parser.add_argument("--no-tcp", action="store_true")
    parser.add_argument("--hz", type=float, default=float(os.environ.get("MUJOCO_SYNC_HZ", "30")))
    args = parser.parse_args()
    use_ros = bool(args.use_ros)

    model_path = Path(args.model)
    if not model_path.is_file():
        raise SystemExit(f"Model not found: {model_path}")

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    joint_to_qpos: dict[str, int] = {}
    name_to_jid: dict[str, int] = {}
    missing = []
    for name in R1LITE_JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            missing.append(name)
            continue
        name_to_jid[name] = int(jid)
        joint_to_qpos[name] = int(model.jnt_qposadr[jid])
    if missing:
        print(f"WARN joints missing in model ({len(missing)}): {missing}", flush=True)
    if not joint_to_qpos:
        raise SystemExit("No R1Lite joints found in MuJoCo model")

    # Ensure a ROS node context exists even for TCP-only (viewer identity in graph).
    try:
        import rclpy
        from rclpy.node import Node

        if not rclpy.ok():
            rclpy.init()

        class Identity(Node):
            def __init__(self) -> None:
                super().__init__(R1LITE_NODE, namespace=R1LITE_NS)

        identity = Identity()
        threading.Thread(target=rclpy.spin, args=(identity,), daemon=True).start()
        print(f"ROS node /{R1LITE_NS}/{R1LITE_NODE} up", flush=True)
    except Exception as exc:
        print(f"ROS identity node skipped: {exc!r}", flush=True)
        identity = None

    store = JointStateStore()
    if use_ros:
        start_ros_subscriber(store, args.topic)
    if not args.no_tcp:
        start_tcp_client(store, args.tcp_host, args.tcp_port)

    period = 1.0 / args.hz if args.hz > 0 else 0.0
    print(
        f"MuJoCo R1Lite sync {args.hz} Hz | model={model_path} | "
        f"mapped={len(joint_to_qpos)} | topic={args.topic} | "
        f"DISPLAY={os.environ.get('DISPLAY', '')}",
        flush=True,
    )

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            latest, source = store.snapshot()
            if latest:
                for name, qadr in joint_to_qpos.items():
                    if name not in latest:
                        continue
                    val = float(latest[name])
                    jid = name_to_jid.get(name, -1)
                    if jid >= 0 and model.jnt_limited[jid]:
                        lo, hi = model.jnt_range[jid]
                        if lo < hi:
                            val = min(max(val, float(lo)), float(hi))
                    data.qpos[qadr] = val
            mujoco.mj_forward(model, data)
            viewer.sync()
            if period:
                time.sleep(period)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
