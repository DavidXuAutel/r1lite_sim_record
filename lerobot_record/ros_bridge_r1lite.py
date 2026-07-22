#!/usr/bin/env python3
"""R1Lite ROS/TCP + camera bridge for episode recording.

Naming (isolated from Franka):
  ROS node: /r1lite/record_bridge
  Joints:   /r1lite/joint_states  OR TCP 10.229.66.95:8765
  Cams:     HTTP MJPEG (default) or /r1lite/cam/<name>/compressed

Never touches bare /joint_states or Franka camera topics.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Any
from urllib.request import urlopen

import cv2
import numpy as np

R1LITE_NS = "r1lite"
R1LITE_NODE = "record_bridge"
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

CAMERAS = ("head", "left_wrist", "right_wrist")
DEFAULT_ROS_CAM = {
    "head": "/r1lite/cam/head/compressed",
    "left_wrist": "/r1lite/cam/left_wrist/compressed",
    "right_wrist": "/r1lite/cam/right_wrist/compressed",
}
DEFAULT_HTTP_BASE = "http://10.229.66.95:8766"
DEFAULT_TCP = ("10.229.66.95", 8765)


@dataclass
class LatestSample:
    q: np.ndarray | None = None
    cams: dict[str, np.ndarray | None] = field(
        default_factory=lambda: {n: None for n in CAMERAS}
    )
    q_stamp: float = 0.0
    cam_stamp: dict[str, float] = field(default_factory=lambda: {n: 0.0 for n in CAMERAS})
    lock: threading.Lock = field(default_factory=threading.Lock)


class R1LiteBridge:
    def __init__(
        self,
        joint_topic: str = R1LITE_JOINT_TOPIC,
        tcp_host: str = DEFAULT_TCP[0],
        tcp_port: int = DEFAULT_TCP[1],
        use_tcp: bool = True,
        use_ros_joints: bool = False,
        cam_transport: str = "http",
        http_base: str = DEFAULT_HTTP_BASE,
        cam_topics: dict[str, str] | None = None,
        stale_s: float = 0.5,
        image_stale_s: float = 1.0,
    ) -> None:
        if joint_topic.rstrip("/") == "/joint_states":
            raise ValueError(
                f"REFUSING bare /joint_states; use {R1LITE_JOINT_TOPIC}"
            )
        self.joint_topic = joint_topic
        self.tcp_host = tcp_host
        self.tcp_port = tcp_port
        self.use_tcp = use_tcp
        self.use_ros_joints = use_ros_joints
        self.cam_transport = cam_transport
        self.http_base = http_base.rstrip("/")
        self.cam_topics = cam_topics or dict(DEFAULT_ROS_CAM)
        self.stale_s = stale_s
        self.image_stale_s = image_stale_s
        self.sample = LatestSample()
        self._node = None
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []

    def start(self) -> None:
        if self._threads:
            return
        self._stop.clear()
        self._start_identity()
        if self.use_ros_joints:
            self._start_ros_joints()
        if self.use_tcp:
            t = threading.Thread(target=self._tcp_loop, name="r1-tcp-joints", daemon=True)
            t.start()
            self._threads.append(t)
        if self.cam_transport == "ros":
            self._start_ros_cams()
        else:
            for name in CAMERAS:
                t = threading.Thread(
                    target=self._http_cam_loop,
                    args=(name,),
                    name=f"r1-http-{name}",
                    daemon=True,
                )
                t.start()
                self._threads.append(t)

    def stop(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2.0)
        self._threads = []

    def _ordered_q(self, mapping: dict[str, float]) -> np.ndarray | None:
        if not all(n in mapping for n in R1LITE_JOINT_NAMES):
            return None
        return np.array([mapping[n] for n in R1LITE_JOINT_NAMES], dtype=np.float32)

    def _update_joints(self, mapping: dict[str, float]) -> None:
        q = self._ordered_q(mapping)
        if q is None:
            return
        with self.sample.lock:
            self.sample.q = q
            self.sample.q_stamp = time.time()

    def _update_cam(self, name: str, bgr: np.ndarray) -> None:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        with self.sample.lock:
            self.sample.cams[name] = rgb
            self.sample.cam_stamp[name] = time.time()

    def _start_identity(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node

            if not rclpy.ok():
                rclpy.init()

            class Identity(Node):
                def __init__(self_inner) -> None:
                    super().__init__(R1LITE_NODE, namespace=R1LITE_NS)

            self._node = Identity()
            t = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
            t.start()
            self._threads.append(t)
            print(f"ROS node /{R1LITE_NS}/{R1LITE_NODE} up", flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"ROS identity skipped: {exc!r}", flush=True)

    def _start_ros_joints(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import JointState

            if not rclpy.ok():
                rclpy.init()
            bridge = self

            class JointSub(Node):
                def __init__(self_inner) -> None:
                    super().__init__("record_joint_sub", namespace=R1LITE_NS)
                    self_inner.create_subscription(
                        JointState,
                        bridge.joint_topic,
                        self_inner._on,
                        qos_profile_sensor_data,
                    )

                def _on(self_inner, msg: JointState) -> None:
                    mapping = {str(n): float(p) for n, p in zip(msg.name, msg.position)}
                    bridge._update_joints(mapping)

            node = JointSub()
            t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
            t.start()
            self._threads.append(t)
            print(f"ROS joints on {self.joint_topic}", flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"ROS joints failed: {exc!r}", flush=True)

    def _tcp_loop(self) -> None:
        while not self._stop.is_set():
            try:
                with socket.create_connection((self.tcp_host, self.tcp_port), timeout=3.0) as sock:
                    sock_file = sock.makefile("r", encoding="utf-8", newline="\n")
                    print(f"TCP joints {self.tcp_host}:{self.tcp_port}", flush=True)
                    for line in sock_file:
                        if self._stop.is_set():
                            break
                        line = line.strip()
                        if not line:
                            continue
                        payload = json.loads(line)
                        names = payload.get("name") or []
                        positions = payload.get("position") or []
                        self._update_joints(
                            {str(n): float(p) for n, p in zip(names, positions)}
                        )
            except Exception as exc:  # noqa: BLE001
                print(f"TCP joints reconnect: {exc!r}", flush=True)
                time.sleep(1.0)

    def _http_cam_loop(self, name: str) -> None:
        url = f"{self.http_base}/stream/{name}"
        while not self._stop.is_set():
            try:
                with urlopen(url, timeout=5) as resp:
                    buf = b""
                    while not self._stop.is_set():
                        chunk = resp.read(4096)
                        if not chunk:
                            break
                        buf += chunk
                        start = buf.find(b"\xff\xd8")
                        end = buf.find(b"\xff\xd9")
                        while start != -1 and end != -1 and end > start:
                            jpg = buf[start : end + 2]
                            buf = buf[end + 2 :]
                            arr = np.frombuffer(jpg, dtype=np.uint8)
                            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                            if img is not None:
                                self._update_cam(name, img)
                            start = buf.find(b"\xff\xd8")
                            end = buf.find(b"\xff\xd9")
            except Exception as exc:  # noqa: BLE001
                print(f"HTTP cam {name} reconnect: {exc!r}", flush=True)
                time.sleep(1.0)

    def _start_ros_cams(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import CompressedImage

            if not rclpy.ok():
                rclpy.init()
            bridge = self

            class CamSub(Node):
                def __init__(self_inner) -> None:
                    super().__init__("record_cam_sub", namespace=R1LITE_NS)
                    for name, topic in bridge.cam_topics.items():
                        self_inner.create_subscription(
                            CompressedImage,
                            topic,
                            lambda msg, n=name: self_inner._on(n, msg),
                            qos_profile_sensor_data,
                        )

                def _on(self_inner, name: str, msg: CompressedImage) -> None:
                    arr = np.frombuffer(msg.data, dtype=np.uint8)
                    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        bridge._update_cam(name, img)

            node = CamSub()
            t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
            t.start()
            self._threads.append(t)
            print(f"ROS cams: {self.cam_topics}", flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"ROS cams failed: {exc!r}", flush=True)

    def get_latest(self) -> dict[str, Any]:
        now = time.time()
        with self.sample.lock:
            q = None if self.sample.q is None else self.sample.q.copy()
            q_stamp = self.sample.q_stamp
            cams = {
                n: (None if self.sample.cams[n] is None else self.sample.cams[n].copy())
                for n in CAMERAS
            }
            cam_stamp = dict(self.sample.cam_stamp)

        ok_q = q is not None and (now - q_stamp) <= self.stale_s
        ok_cams = {
            n: cams[n] is not None and (now - cam_stamp[n]) <= self.image_stale_s
            for n in CAMERAS
        }
        ok = ok_q and all(ok_cams.values())
        return {
            "ok": ok,
            "ok_joints": ok_q,
            "ok_head": ok_cams["head"],
            "ok_left_wrist": ok_cams["left_wrist"],
            "ok_right_wrist": ok_cams["right_wrist"],
            "q": q if q is not None else np.zeros(len(R1LITE_JOINT_NAMES), dtype=np.float32),
            "head": cams["head"],
            "left_wrist": cams["left_wrist"],
            "right_wrist": cams["right_wrist"],
            "age": {
                "joints": None if q is None else now - q_stamp,
                **{n: (None if cams[n] is None else now - cam_stamp[n]) for n in CAMERAS},
            },
        }
