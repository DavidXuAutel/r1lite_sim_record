#!/usr/bin/env python3
"""AHA-WAM ↔ R1Lite policy bridge (runs on robot PC).

Observation:
  - front RGB: MJPEG head stream (or ROS compressed)
  - state14: from /joint_states (vendor) via ordered pack

Inference:
  - TCP pickle to policy server (or --mock-policy hold pose)

Command (optional, OFF by default):
  - /motion_target/target_joint_state_arm_{left,right}  sensor_msgs/JointState
  - /motion_control/position_control_gripper_{left,right}  std_msgs/Float32
  - optional --cmd-mode hdas → hdas_msg/MotorControl on /motion_control/control_arm_*

Also streams session samples to host recorder (JSON lines TCP):
  {"t":..., "state":[14], "action":[14], "chunk_i":int, "source":"policy|mock"}

ROS node: /r1lite/policy_bridge
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from deploy.r1lite_dof import (  # noqa: E402
    LEFT_ARM_CMD_NAMES,
    RIGHT_ARM_CMD_NAMES,
    clip_delta,
    pack_state14,
    unpack_action14,
)
from deploy.tcp_protocol import PolicyClient  # noqa: E402

os.environ.setdefault("ROS2CLI_DISABLE_DAEMON", "1")


class Latest:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.q: dict[str, float] = {}
        self.q_ts = 0.0
        self.rgb: np.ndarray | None = None
        self.rgb_ts = 0.0

    def set_joints(self, mapping: dict[str, float]) -> None:
        with self.lock:
            self.q.update(mapping)
            self.q_ts = time.time()

    def set_rgb(self, rgb: np.ndarray) -> None:
        with self.lock:
            self.rgb = rgb
            self.rgb_ts = time.time()

    def snapshot(self):
        with self.lock:
            q = dict(self.q)
            rgb = None if self.rgb is None else self.rgb.copy()
            return q, self.q_ts, rgb, self.rgb_ts


def _http_cam_loop(store: Latest, base: str, name: str = "head") -> None:
    import cv2
    from urllib.request import urlopen

    url = f"{base.rstrip('/')}/stream/{name}"
    while True:
        try:
            with urlopen(url, timeout=5) as resp:
                buf = b""
                while True:
                    chunk = resp.read(4096)
                    if not chunk:
                        break
                    buf += chunk
                    start, end = buf.find(b"\xff\xd8"), buf.find(b"\xff\xd9")
                    while start != -1 and end != -1 and end > start:
                        jpg = buf[start : end + 2]
                        buf = buf[end + 2 :]
                        arr = np.frombuffer(jpg, dtype=np.uint8)
                        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if bgr is not None:
                            store.set_rgb(bgr[:, :, ::-1].copy())
                        start, end = buf.find(b"\xff\xd8"), buf.find(b"\xff\xd9")
        except Exception as exc:  # noqa: BLE001
            print(f"[cam] reconnect: {exc!r}", flush=True)
            time.sleep(1.0)


def _session_push(host: str, port: int, sample: dict) -> None:
    if not host or port <= 0:
        return
    try:
        with socket.create_connection((host, port), timeout=0.5) as sock:
            sock.sendall((json.dumps(sample) + "\n").encode("utf-8"))
    except Exception:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    # Defaults assume bridge runs ON the robot with local SSH tunnel to policy.
    parser.add_argument("--server-ip", default="127.0.0.1")
    parser.add_argument("--server-port", type=int, default=10000)
    parser.add_argument("--instruction", default="pick up the red block")
    parser.add_argument("--http-cam", default="http://127.0.0.1:8766")
    parser.add_argument("--mock-policy", action="store_true", help="hold current state as action chunk")
    parser.add_argument("--enable-cmd", action="store_true", help="publish motion targets (DANGEROUS)")
    parser.add_argument("--cmd-mode", choices=("target", "hdas"), default="target")
    parser.add_argument("--max-joint-delta", type=float, default=0.05)
    parser.add_argument("--max-gripper-delta", type=float, default=0.01)
    parser.add_argument("--infer-hz", type=float, default=1.0)
    parser.add_argument("--control-hz", type=float, default=30.0)
    parser.add_argument("--chunk-dt", type=float, default=0.1, help="seconds between chunk waypoints")
    parser.add_argument("--session-host", default="10.229.20.125", help="125 session recorder")
    parser.add_argument("--session-port", type=int, default=8777)
    parser.add_argument("--stale-s", type=float, default=1.0)
    args = parser.parse_args()

    if args.enable_cmd:
        print("!!! --enable-cmd: will publish joint targets to the robot !!!", flush=True)
    else:
        print("[safe] dry-run: inference only, no robot commands", flush=True)

    store = Latest()
    threading.Thread(target=_http_cam_loop, args=(store, args.http_cam), daemon=True).start()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32

    qos_hdas = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    class Bridge(Node):
        def __init__(self) -> None:
            super().__init__("policy_bridge", namespace="r1lite")
            self.create_subscription(JointState, "/joint_states", self._on_js, qos_hdas)
            # Fallback when jointTracker /joint_states is empty: HDAS feedback
            self.create_subscription(JointState, "/hdas/feedback_arm_left", self._on_left, qos_hdas)
            self.create_subscription(JointState, "/hdas/feedback_arm_right", self._on_right, qos_hdas)
            self.create_subscription(JointState, "/hdas/feedback_gripper_left", self._on_g_left, qos_hdas)
            self.create_subscription(JointState, "/hdas/feedback_gripper_right", self._on_g_right, qos_hdas)
            self.pub_left = self.create_publisher(JointState, "/motion_target/target_joint_state_arm_left", 10)
            self.pub_right = self.create_publisher(JointState, "/motion_target/target_joint_state_arm_right", 10)
            self.pub_g_l = self.create_publisher(Float32, "/motion_control/position_control_gripper_left", 10)
            self.pub_g_r = self.create_publisher(Float32, "/motion_control/position_control_gripper_right", 10)
            self.motor_left = self.motor_right = None
            if args.cmd_mode == "hdas":
                from hdas_msg.msg import MotorControl

                self.motor_left = self.create_publisher(MotorControl, "/motion_control/control_arm_left", 10)
                self.motor_right = self.create_publisher(MotorControl, "/motion_control/control_arm_right", 10)

        def _on_js(self, msg: JointState) -> None:
            store.set_joints({str(n): float(p) for n, p in zip(msg.name, msg.position)})

        def _on_left(self, msg: JointState) -> None:
            # Map positional order → left_arm_joint1..6 when names missing/opaque
            mapping = {}
            if len(msg.position) >= 6:
                for i, n in enumerate(LEFT_ARM_CMD_NAMES):
                    mapping[n] = float(msg.position[i])
            for n, p in zip(msg.name, msg.position):
                mapping[str(n)] = float(p)
            store.set_joints(mapping)

        def _on_right(self, msg: JointState) -> None:
            mapping = {}
            if len(msg.position) >= 6:
                for i, n in enumerate(RIGHT_ARM_CMD_NAMES):
                    mapping[n] = float(msg.position[i])
            for n, p in zip(msg.name, msg.position):
                mapping[str(n)] = float(p)
            store.set_joints(mapping)

        def _on_g_left(self, msg: JointState) -> None:
            # HDAS single scalar is ~0..100 open; finger joints are meters.
            mapping = {}
            if len(msg.position) >= 2:
                mapping["left_gripper_finger_joint1"] = float(msg.position[0])
                mapping["left_gripper_finger_joint2"] = float(msg.position[1])
            elif len(msg.position) == 1:
                open_frac = max(0.0, min(1.0, float(msg.position[0]) / 100.0))
                mapping["left_gripper_finger_joint1"] = open_frac * 0.05
                mapping["left_gripper_finger_joint2"] = -open_frac * 0.05
            store.set_joints(mapping)

        def _on_g_right(self, msg: JointState) -> None:
            mapping = {}
            if len(msg.position) >= 2:
                mapping["right_gripper_finger_joint1"] = float(msg.position[0])
                mapping["right_gripper_finger_joint2"] = float(msg.position[1])
            elif len(msg.position) == 1:
                open_frac = max(0.0, min(1.0, float(msg.position[0]) / 100.0))
                mapping["right_gripper_finger_joint1"] = open_frac * 0.05
                mapping["right_gripper_finger_joint2"] = -open_frac * 0.05
            store.set_joints(mapping)

        def publish_action14(self, a14: np.ndarray) -> None:
            parts = unpack_action14(a14)
            now = self.get_clock().now().to_msg()
            left = JointState()
            left.header.stamp = now
            left.name = list(LEFT_ARM_CMD_NAMES)
            left.position = [parts[n] for n in LEFT_ARM_CMD_NAMES]
            right = JointState()
            right.header.stamp = now
            right.name = list(RIGHT_ARM_CMD_NAMES)
            right.position = [parts[n] for n in RIGHT_ARM_CMD_NAMES]
            self.pub_left.publish(left)
            self.pub_right.publish(right)
            # Policy gripper ≈ meters / Piper open width; HDAS position_control uses ~0..100.
            def _grip_cmd(g: float) -> float:
                # Map roughly [0, 0.05]m -> [0, 100]; clamp for safety.
                return float(max(0.0, min(100.0, (float(g) / 0.05) * 100.0)))

            self.pub_g_l.publish(Float32(data=_grip_cmd(parts["left_gripper"])))
            self.pub_g_r.publish(Float32(data=_grip_cmd(parts["right_gripper"])))
            if self.motor_left is not None:
                from hdas_msg.msg import MotorControl

                for pub, names in (
                    (self.motor_left, LEFT_ARM_CMD_NAMES),
                    (self.motor_right, RIGHT_ARM_CMD_NAMES),
                ):
                    m = MotorControl()
                    m.header.stamp = now
                    m.name = ""
                    m.p_des = [float(parts[n]) for n in names]
                    m.v_des = [0.0] * 6
                    m.kp = [0.0] * 6
                    m.kd = [0.0] * 6
                    m.t_ff = [0.0] * 6
                    m.mode = 1
                    pub.publish(m)

    rclpy.init()
    node = Bridge()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print("ROS node /r1lite/policy_bridge up", flush=True)

    client = None if args.mock_policy else PolicyClient(args.server_ip, args.server_port)
    infer_period = 1.0 / max(args.infer_hz, 0.1)
    ctrl_period = 1.0 / max(args.control_hz, 1.0)

    # Wait for streams
    t_end = time.time() + 30.0
    state = None
    rgb = None
    while time.time() < t_end:
        q, qts, rgb, rts = store.snapshot()
        state = pack_state14(q)
        if state is not None and rgb is not None and (time.time() - qts) < args.stale_s and (time.time() - rts) < args.stale_s:
            break
        time.sleep(0.1)
    else:
        raise SystemExit("streams not ready (joints/camera)")

    active_chunk: np.ndarray | None = None
    chunk_t0 = 0.0
    last_cmd = state.copy()
    next_infer = 0.0

    try:
        while rclpy.ok():
            now = time.time()
            q, qts, rgb, rts = store.snapshot()
            state = pack_state14(q)
            if state is None or rgb is None:
                time.sleep(0.05)
                continue
            if (now - qts) > args.stale_s or (now - rts) > args.stale_s:
                time.sleep(0.05)
                continue

            if now >= next_infer:
                next_infer = now + infer_period
                try:
                    if args.mock_policy:
                        chunk = np.tile(state[None, :], (16, 1)).astype(np.float32)
                        lat = 0.0
                        step = -1
                        source = "mock"
                    else:
                        assert client is not None
                        resp = client.infer(args.instruction, state, rgb)
                        if not resp.get("ok"):
                            print(f"[infer bad] {resp!r}", flush=True)
                            continue
                        chunk = np.asarray(resp["action_chunk"], dtype=np.float32)
                        lat = float(resp.get("model_latency_ms") or 0.0)
                        step = int(resp.get("server_inference_step") or -1)
                        source = "policy"
                    if chunk.ndim != 2 or chunk.shape[1] != 14:
                        print(f"[infer] bad chunk shape {chunk.shape}", flush=True)
                        continue
                    active_chunk = chunk
                    chunk_t0 = now
                    print(
                        f"[infer] source={source} shape={chunk.shape} model_ms={lat:.0f} step={step} "
                        f"a0={np.array2string(chunk[0], precision=3)}",
                        flush=True,
                    )
                    _session_push(
                        args.session_host,
                        args.session_port,
                        {
                            "t": now,
                            "type": "chunk",
                            "source": source,
                            "state": state.tolist(),
                            "action_chunk": chunk.tolist(),
                            "model_latency_ms": lat,
                            "server_inference_step": step,
                            "instruction": args.instruction,
                        },
                    )
                except Exception as exc:  # noqa: BLE001
                    print(f"[infer error] {exc!r}", flush=True)

            # Execute / log current waypoint from chunk
            if active_chunk is not None:
                idx = int((now - chunk_t0) / max(args.chunk_dt, 1e-3))
                idx = min(max(idx, 0), active_chunk.shape[0] - 1)
                target = active_chunk[idx]
                cmd = clip_delta(last_cmd, target, args.max_joint_delta, args.max_gripper_delta)
                last_cmd = cmd
                _session_push(
                    args.session_host,
                    args.session_port,
                    {
                        "t": now,
                        "type": "step",
                        "state": state.tolist(),
                        "action": cmd.tolist(),
                        "chunk_i": idx,
                    },
                )
                if args.enable_cmd:
                    node.publish_action14(cmd)

            time.sleep(ctrl_period)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
