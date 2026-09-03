#!/usr/bin/env python3
"""R1Lite → AHA-WAM policy dry-run client (TCP + pickle).

Reads:
  - head RGB from MJPEG  http://<robot>:8766/stream/head
  - joints from TCP      <robot>:8765  (or optional /r1lite/joint_states)

Sends infer requests; prints action_chunk stats. NEVER publishes robot commands.

Protocol: 4-byte big-endian length + pickle payload.
"""

from __future__ import annotations

import argparse
import pickle
import socket
import struct
import threading
import time
from typing import Any
from urllib.request import urlopen

import numpy as np

# RoboTwin 14-DoF: L6 + Lg + R6 + Rg
LEFT_ARM = [f"left_arm_joint{i}" for i in range(1, 7)]
RIGHT_ARM = [f"right_arm_joint{i}" for i in range(1, 7)]
LEFT_FINGERS = ("left_gripper_finger_joint1", "left_gripper_finger_joint2")
RIGHT_FINGERS = ("right_gripper_finger_joint1", "right_gripper_finger_joint2")


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed")
        buf += chunk
    return buf


def send_msg(sock: socket.socket, obj: Any) -> None:
    payload = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
    sock.sendall(struct.pack(">I", len(payload)) + payload)


def recv_msg(sock: socket.socket) -> Any:
    (length,) = struct.unpack(">I", _recv_exact(sock, 4))
    return pickle.loads(_recv_exact(sock, length))


class JointStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._q: dict[str, float] = {}
        self._ts = 0.0

    def update(self, mapping: dict[str, float]) -> None:
        with self._lock:
            self._q.update(mapping)
            self._ts = time.time()

    def state14(self) -> tuple[np.ndarray | None, float]:
        with self._lock:
            q = dict(self._q)
            age = time.time() - self._ts if self._ts else 1e9
        need = LEFT_ARM + list(LEFT_FINGERS) + RIGHT_ARM + list(RIGHT_FINGERS)
        if not all(n in q for n in need):
            return None, age
        lg = 0.5 * (q[LEFT_FINGERS[0]] + q[LEFT_FINGERS[1]])
        rg = 0.5 * (q[RIGHT_FINGERS[0]] + q[RIGHT_FINGERS[1]])
        vec = np.array(
            [q[n] for n in LEFT_ARM] + [lg] + [q[n] for n in RIGHT_ARM] + [rg],
            dtype=np.float32,
        )
        return vec, age


class FrameStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._rgb: np.ndarray | None = None
        self._ts = 0.0

    def set_bgr(self, bgr: np.ndarray) -> None:
        rgb = bgr[:, :, ::-1].copy()
        with self._lock:
            self._rgb = rgb
            self._ts = time.time()

    def get(self) -> tuple[np.ndarray | None, float]:
        with self._lock:
            if self._rgb is None:
                return None, 1e9
            return self._rgb.copy(), time.time() - self._ts


def _tcp_joints(store: JointStore, host: str, port: int) -> None:
    while True:
        try:
            with socket.create_connection((host, port), timeout=3.0) as sock:
                f = sock.makefile("r", encoding="utf-8", newline="\n")
                print(f"[joints] connected {host}:{port}", flush=True)
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    import json

                    payload = json.loads(line)
                    names = payload.get("name") or []
                    pos = payload.get("position") or []
                    store.update({str(n): float(p) for n, p in zip(names, pos)})
        except Exception as exc:  # noqa: BLE001
            print(f"[joints] reconnect: {exc!r}", flush=True)
            time.sleep(1.0)


def _http_cam(store: FrameStore, base: str, name: str = "head") -> None:
    import cv2

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
                    start = buf.find(b"\xff\xd8")
                    end = buf.find(b"\xff\xd9")
                    while start != -1 and end != -1 and end > start:
                        jpg = buf[start : end + 2]
                        buf = buf[end + 2 :]
                        arr = np.frombuffer(jpg, dtype=np.uint8)
                        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if img is not None:
                            store.set_bgr(img)
                        start = buf.find(b"\xff\xd8")
                        end = buf.find(b"\xff\xd9")
        except Exception as exc:  # noqa: BLE001
            print(f"[cam] reconnect: {exc!r}", flush=True)
            time.sleep(1.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    # Defaults: run ON r1lite with local policy tunnel + local relays.
    parser.add_argument("--server-ip", default="127.0.0.1")
    parser.add_argument("--server-port", type=int, default=10000)
    parser.add_argument("--instruction", default="pick up the red block")
    parser.add_argument("--http-cam", default="http://127.0.0.1:8766")
    parser.add_argument("--tcp-joints", default="127.0.0.1:8765")
    parser.add_argument("--rate", type=float, default=1.0, help="infer Hz (dry-run; keep low)")
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    jhost, jport_s = args.tcp_joints.rsplit(":", 1)
    jport = int(jport_s)

    joints = JointStore()
    frames = FrameStore()
    threading.Thread(target=_tcp_joints, args=(joints, jhost, jport), daemon=True).start()
    threading.Thread(target=_http_cam, args=(frames, args.http_cam), daemon=True).start()

    print(
        f"[dry-run] policy={args.server_ip}:{args.server_port} "
        f"cam={args.http_cam}/stream/head joints={args.tcp_joints}",
        flush=True,
    )
    print("[dry-run] will NOT send any robot commands", flush=True)

    deadline = time.time() + 30.0
    while time.time() < deadline:
        st, ja = joints.state14()
        img, ia = frames.get()
        if st is not None and img is not None and ja < 1.0 and ia < 1.0:
            break
        time.sleep(0.2)
    else:
        st, ja = joints.state14()
        img, ia = frames.get()
        raise SystemExit(
            f"streams not ready: state={st is not None} age={ja:.2f} "
            f"img={img is not None} age={ia:.2f}"
        )

    period = 1.0 / max(args.rate, 0.1)
    while True:
        st, ja = joints.state14()
        img, ia = frames.get()
        if st is None or img is None:
            print(f"[skip] stale joints_age={ja:.2f} img_age={ia:.2f}", flush=True)
            time.sleep(0.5)
            continue

        req = {
            "type": "infer",
            "instruction": args.instruction,
            "state": st,
            "images": {"front": img},
        }
        t0 = time.perf_counter()
        try:
            with socket.create_connection((args.server_ip, args.server_port), timeout=10.0) as sock:
                sock.settimeout(120.0)
                send_msg(sock, req)
                resp = recv_msg(sock)
        except Exception as exc:  # noqa: BLE001
            print(f"[error] {exc!r}", flush=True)
            if args.once:
                return 1
            time.sleep(2.0)
            continue

        dt = (time.perf_counter() - t0) * 1000.0
        if not isinstance(resp, dict) or not resp.get("ok"):
            print(f"[bad] {resp!r}", flush=True)
        else:
            chunk = np.asarray(resp["action_chunk"], dtype=np.float32)
            print(
                f"[ok] shape={chunk.shape} rtt_ms={dt:.0f} "
                f"model_ms={resp.get('model_latency_ms')} "
                f"step={resp.get('server_inference_step')} "
                f"action[0]={np.array2string(chunk[0], precision=3, separator=',')}",
                flush=True,
            )
        if args.once:
            return 0
        time.sleep(period)


if __name__ == "__main__":
    raise SystemExit(main())
