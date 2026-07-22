#!/usr/bin/env python3
"""R1Lite three-camera OpenCV viewer with LeRobot START/STOP/REPLAY/ABORT.

Naming (isolated from Franka):
  ROS node: /r1lite/cam_view
  Record API: http://127.0.0.1:8775  (Franka uses 8765)
  Dataset: /home/yao/r1lite_lerobot_datasets/<repo>/
  Window: R1Lite Record | head + left_wrist + right_wrist

Views (left -> right): head | left_wrist | right_wrist
Transport:
  --transport http  (default): MJPEG from robot camera_mjpeg
  --transport ros:   CompressedImage on /r1lite/cam/<name>/compressed
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from urllib.request import urlopen

import cv2
import numpy as np

os.environ.setdefault("DISPLAY", ":1")

R1LITE_NS = "r1lite"
R1LITE_NODE = "cam_view"

CAMERAS = ("head", "left_wrist", "right_wrist")
DEFAULT_ROS_TOPICS = {
    "head": "/r1lite/cam/head/compressed",
    "left_wrist": "/r1lite/cam/left_wrist/compressed",
    "right_wrist": "/r1lite/cam/right_wrist/compressed",
}
DEFAULT_HTTP_BASE = "http://10.229.66.95:8766"
DEFAULT_API = "http://127.0.0.1:8775"
DEFAULT_REPO = "r1lite_teleop"
DEFAULT_DATA_ROOT = "/home/yao/r1lite_lerobot_datasets"

BAR_H = 84
BTN_W = 120
BTN_H = 48
BTN_GAP = 12


class FrameStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frames: dict[str, tuple[np.ndarray, float]] = {}

    def set_bgr(self, name: str, bgr: np.ndarray) -> None:
        with self._lock:
            self._frames[name] = (bgr, time.time())

    def snapshot(self) -> dict[str, tuple[np.ndarray | None, float]]:
        with self._lock:
            out: dict[str, tuple[np.ndarray | None, float]] = {}
            for name in CAMERAS:
                row = self._frames.get(name)
                if row is None:
                    out[name] = (None, 0.0)
                else:
                    out[name] = (row[0].copy(), row[1])
            return out


class RecordClient:
    def __init__(self, base: str, repo: str, task: str) -> None:
        self.base = base.rstrip("/")
        self.repo = repo
        self.task = task
        self._lock = threading.Lock()
        self.status: dict = {"recording": False, "frames": 0}
        self.last_msg = ""
        self._busy = False

    def refresh(self) -> None:
        try:
            with urllib.request.urlopen(f"{self.base}/record/status", timeout=1.5) as resp:
                data = json.loads(resp.read().decode())
            with self._lock:
                self.status = data
                if self.last_msg.startswith("status err:"):
                    self.last_msg = ""
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self.last_msg = f"status err: {exc}"

    def start(self) -> None:
        self._post("/record/start", {"repo": self.repo, "task": self.task})

    def stop(self) -> None:
        self._post("/record/stop", {})

    def _post(self, path: str, body: dict) -> None:
        with self._lock:
            if self._busy:
                return
            self._busy = True
            self.last_msg = "..."

        def _run() -> None:
            try:
                payload = json.dumps(body).encode()
                req = urllib.request.Request(
                    f"{self.base}{path}",
                    data=payload,
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with urllib.request.urlopen(req, timeout=120) as resp:
                    data = json.loads(resp.read().decode())
                with self._lock:
                    self.status = data
                    if data.get("recording"):
                        self.last_msg = f"REC frames={data.get('frames', 0)}"
                    else:
                        self.last_msg = f"stopped frames={data.get('frames', 0)}"
            except urllib.error.HTTPError as exc:
                detail = exc.read().decode(errors="replace")
                try:
                    detail = json.loads(detail).get("detail", detail)
                except Exception:  # noqa: BLE001
                    pass
                with self._lock:
                    self.last_msg = f"HTTP {exc.code}: {str(detail)[:140]}"
            except Exception as exc:  # noqa: BLE001
                with self._lock:
                    self.last_msg = f"error: {exc}"
            finally:
                with self._lock:
                    self._busy = False
                self.refresh()

        threading.Thread(target=_run, daemon=True).start()


def _http_worker(store: FrameStore, base: str, name: str) -> None:
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
                            store.set_bgr(name, img)
                        start = buf.find(b"\xff\xd8")
                        end = buf.find(b"\xff\xd9")
        except Exception as exc:  # noqa: BLE001
            print(f"HTTP {name} reconnect: {exc!r}", flush=True)
            time.sleep(1.0)


def _start_ros_subscribers(store: FrameStore, topics: dict[str, str]) -> object | None:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage
    except Exception as exc:  # noqa: BLE001
        print(f"ROS unavailable: {exc!r}", flush=True)
        return None

    class CamViewNode(Node):
        def __init__(self) -> None:
            super().__init__(R1LITE_NODE, namespace=R1LITE_NS)
            for name, topic in topics.items():
                self.create_subscription(
                    CompressedImage,
                    topic,
                    lambda msg, n=name: self._on_compressed(n, msg),
                    qos_profile_sensor_data,
                )
                self.get_logger().info(f"sub {topic}")

        def _on_compressed(self, name: str, msg: CompressedImage) -> None:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is not None:
                store.set_bgr(name, img)

    if not rclpy.ok():
        rclpy.init()
    node = CamViewNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print(f"ROS node /{R1LITE_NS}/{R1LITE_NODE} up", flush=True)
    return node


def _start_ros_identity() -> object | None:
    try:
        import rclpy
        from rclpy.node import Node

        if not rclpy.ok():
            rclpy.init()

        class Identity(Node):
            def __init__(self) -> None:
                super().__init__(R1LITE_NODE, namespace=R1LITE_NS)

        node = Identity()
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        print(f"ROS node /{R1LITE_NS}/{R1LITE_NODE} up (http transport)", flush=True)
        return node
    except Exception as exc:  # noqa: BLE001
        print(f"ROS identity skipped: {exc!r}", flush=True)
        return None


def _btn_rects(canvas_w: int) -> dict[str, tuple[int, int, int, int]]:
    y1 = 10
    y2 = y1 + BTN_H
    x = 16
    boxes = {}
    for name in ("start", "stop", "replay", "abort"):
        boxes[name] = (x, y1, x + BTN_W, y2)
        x += BTN_W + BTN_GAP
    return boxes


def _draw_button(bar: np.ndarray, box: tuple[int, int, int, int], label: str, color, enabled: bool) -> None:
    x1, y1, x2, y2 = box
    fill = color if enabled else (80, 80, 80)
    cv2.rectangle(bar, (x1, y1), (x2, y2), fill, thickness=-1)
    cv2.rectangle(bar, (x1, y1), (x2, y2), (255, 255, 255), thickness=2)
    tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)[0][0]
    tx = x1 + max(4, (BTN_W - tw) // 2)
    ty = y1 + 32
    cv2.putText(bar, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)


def _dataset_root(repo: str) -> Path:
    base = os.environ.get("LOCAL_DATASET_ROOT", DEFAULT_DATA_ROOT)
    return Path(base) / repo


def _abort_r1lite_stack() -> str:
    """Stop only R1Lite view/record/mirror — never Franka teleop."""
    root = Path(__file__).resolve().parents[1]
    log_dir = Path(os.environ.get("R1LITE_LOG_DIR", str(root / "logs")))
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / "cam_view_abort.log"
    daemon = root / "lerobot_record" / "r1lite_record_daemon.sh"
    try:
        with open(log_path, "ab", buffering=0) as log_f:
            if daemon.is_file():
                subprocess.run(
                    ["bash", str(daemon), "stop"],
                    stdout=log_f,
                    stderr=log_f,
                    timeout=30,
                    check=False,
                )
            subprocess.run(
                [
                    "bash",
                    "-lc",
                    "pkill -f 'cam_view_triple.py|r1lite_record_server.py|"
                    "mujoco_ros_mirror_r1lite.py|cam_replay_r1lite.py' 2>/dev/null || true",
                ],
                stdout=log_f,
                stderr=log_f,
                timeout=15,
                check=False,
            )
        return f"ABORT: R1Lite stack stopping (log {log_path})"
    except Exception as exc:  # noqa: BLE001
        return f"ABORT failed: {exc}"


def _launch_replay(repo: str) -> str:
    root = _dataset_root(repo)
    script = Path(__file__).resolve().parents[1] / "lerobot_record" / "cam_replay_r1lite.py"
    if not script.is_file():
        return f"missing {script}"
    if not (root / "videos").is_dir():
        return f"no videos yet under {root}"
    env = os.environ.copy()
    env["DISPLAY"] = os.environ.get("DISPLAY") or ":1"
    log_dir = Path(os.environ.get("R1LITE_LOG_DIR", str(Path(__file__).resolve().parents[1] / "logs")))
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / "cam_replay_r1lite.log"
    log_f = open(log_path, "ab", buffering=0)  # noqa: SIM115
    py = "/usr/bin/python3" if Path("/usr/bin/python3").is_file() else sys.executable
    proc = subprocess.Popen(
        [py, str(script), "--root", str(root), "--loop"],
        env=env,
        stdout=log_f,
        stderr=log_f,
        start_new_session=True,
    )
    return f"REPLAY pid={proc.pid} log={log_path}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transport", choices=("http", "ros"), default="http")
    parser.add_argument("--http-base", default=os.environ.get("R1LITE_CAM_HTTP", DEFAULT_HTTP_BASE))
    parser.add_argument("--scale", type=float, default=0.75)
    parser.add_argument("--head", default=DEFAULT_ROS_TOPICS["head"])
    parser.add_argument("--left-wrist", default=DEFAULT_ROS_TOPICS["left_wrist"])
    parser.add_argument("--right-wrist", default=DEFAULT_ROS_TOPICS["right_wrist"])
    parser.add_argument("--api", default=os.environ.get("R1LITE_RECORD_API", DEFAULT_API))
    parser.add_argument("--repo", default=os.environ.get("R1LITE_REPO", DEFAULT_REPO))
    parser.add_argument("--task", default=os.environ.get("R1LITE_TASK", "r1lite teleop"))
    parser.add_argument(
        "--data-root",
        default=os.environ.get("LOCAL_DATASET_ROOT", DEFAULT_DATA_ROOT),
        help="Local LeRobot datasets parent directory (R1Lite)",
    )
    args = parser.parse_args()
    os.environ["LOCAL_DATASET_ROOT"] = args.data_root

    client = RecordClient(args.api, args.repo, args.task)
    client.refresh()

    store = FrameStore()
    if args.transport == "ros":
        topics = {
            "head": args.head,
            "left_wrist": args.left_wrist,
            "right_wrist": args.right_wrist,
        }
        _start_ros_subscribers(store, topics)
    else:
        _start_ros_identity()
        for name in CAMERAS:
            threading.Thread(
                target=_http_worker, args=(store, args.http_base, name), daemon=True
            ).start()

    win = "R1Lite Record | head + left_wrist + right_wrist"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    print(f"window={win} transport={args.transport} api={args.api}", flush=True)

    click = {"action": None}
    aborting = False

    def on_mouse(event, x, y, _flags, _param) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        h = getattr(on_mouse, "canvas_h", 0)
        if h <= 0 or y < h - BAR_H:
            return
        local_y = y - (h - BAR_H)
        rects = _btn_rects(getattr(on_mouse, "canvas_w", 800))
        for name, (x1, y1, x2, y2) in rects.items():
            if x1 <= x <= x2 and y1 <= local_y <= y2:
                click["action"] = name
                break

    cv2.setMouseCallback(win, on_mouse)
    last_poll = 0.0
    data_path = str(_dataset_root(args.repo))

    try:
        while not aborting:
            now = time.time()
            if now - last_poll > 0.5:
                client.refresh()
                last_poll = now

            action = click["action"]
            if action == "start":
                click["action"] = None
                client.start()
            elif action == "stop":
                click["action"] = None
                client.stop()
            elif action == "replay":
                click["action"] = None
                with client._lock:
                    client.last_msg = _launch_replay(args.repo)
            elif action == "abort":
                click["action"] = None
                with client._lock:
                    recording_now = bool(client.status.get("recording"))
                    client.last_msg = "ABORT: shutting down..."
                if recording_now:
                    try:
                        client.stop()
                        time.sleep(0.3)
                    except Exception:  # noqa: BLE001
                        pass
                msg = _abort_r1lite_stack()
                with client._lock:
                    client.last_msg = msg
                aborting = True
                break

            snap = store.snapshot()
            panels = []
            for name in CAMERAS:
                img, ts = snap[name]
                if img is None:
                    panel = np.zeros((360, 640, 3), dtype=np.uint8)
                    text = f"{name}: waiting"
                else:
                    panel = img
                    age = now - ts if ts else -1.0
                    text = f"{name}: {panel.shape[1]}x{panel.shape[0]} age={age:.2f}s"
                cv2.putText(panel, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                if args.scale != 1.0:
                    panel = cv2.resize(panel, None, fx=args.scale, fy=args.scale)
                panels.append(panel)

            h = max(p.shape[0] for p in panels)
            fixed = []
            for p in panels:
                if p.shape[0] != h:
                    p = cv2.resize(p, (int(p.shape[1] * h / p.shape[0]), h))
                fixed.append(p)
            canvas = np.hstack(fixed)
            bar = np.zeros((BAR_H, canvas.shape[1], 3), dtype=np.uint8)
            bar[:] = (40, 40, 40)

            with client._lock:
                recording = bool(client.status.get("recording"))
                frames = int(client.status.get("frames") or 0)
                msg = client.last_msg
                err = client.status.get("last_error")
                streams = client.status.get("streams") or {}
                stream_ok = streams.get("ok")
                busy = client._busy

            rects = _btn_rects(canvas.shape[1])
            _draw_button(bar, rects["start"], "START", (0, 160, 0), enabled=not recording and not busy)
            _draw_button(bar, rects["stop"], "STOP", (0, 0, 200), enabled=recording and not busy)
            _draw_button(bar, rects["replay"], "REPLAY", (180, 120, 0), enabled=not recording and not busy)
            _draw_button(bar, rects["abort"], "ABORT", (0, 0, 120), enabled=True)

            status = f"{'REC' if recording else 'IDLE'} frames={frames} streams={'OK' if stream_ok else '?'}"
            if busy:
                status = "BUSY " + status
            color = (0, 0, 255) if recording else (200, 200, 200)
            x_info = 16 + 4 * (BTN_W + BTN_GAP)
            cv2.putText(bar, status, (x_info, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            cv2.putText(
                bar,
                f"save: {data_path}",
                (x_info, 52),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (170, 170, 170),
                1,
            )
            tip = msg or (f"ERR: {err}" if err else "ABORT=stop R1Lite view/record only")
            tip_color = (
                (0, 0, 255)
                if (
                    err
                    or tip.startswith("HTTP")
                    or tip.startswith("error")
                    or tip.startswith("status err")
                    or tip.startswith("ABORT")
                )
                else (180, 180, 180)
            )
            cv2.putText(bar, tip[:90], (x_info, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.4, tip_color, 1)

            out = np.vstack([canvas, bar])
            on_mouse.canvas_h = out.shape[0]
            on_mouse.canvas_w = out.shape[1]
            cv2.imshow(win, out)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                client.start()
            if key == ord("e"):
                client.stop()
            if key == ord("p"):
                with client._lock:
                    client.last_msg = _launch_replay(args.repo)
            if key == ord("x"):
                click["action"] = "abort"
    finally:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
