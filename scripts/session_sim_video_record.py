#!/usr/bin/env python3
"""On 125: record R1Lite triple-cam MP4 + joint/action sidecar while MuJoCo mirrors.

Listens:
  - MJPEG cams from robot :8766
  - joint TCP :8765
  - optional policy session TCP :8777 (JSON lines from policy_bridge)

Writes under --out-dir/<timestamp>/:
  videos/{head,left_wrist,right_wrist}.mp4
  meta/joints.jsonl
  meta/actions.jsonl
  meta/info.json
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import threading
import time
from datetime import datetime
from pathlib import Path
from urllib.request import urlopen

import cv2
import numpy as np

CAMERAS = ("head", "left_wrist", "right_wrist")


class MjpegReader:
    def __init__(self, base: str, name: str) -> None:
        self.url = f"{base.rstrip('/')}/stream/{name}"
        self.lock = threading.Lock()
        self.frame: np.ndarray | None = None
        self.ts = 0.0
        self._stop = threading.Event()

    def start(self) -> None:
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self) -> None:
        self._stop.set()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                with urlopen(self.url, timeout=5) as resp:
                    buf = b""
                    while not self._stop.is_set():
                        chunk = resp.read(4096)
                        if not chunk:
                            break
                        buf += chunk
                        start, end = buf.find(b"\xff\xd8"), buf.find(b"\xff\xd9")
                        while start != -1 and end != -1 and end > start:
                            jpg = buf[start : end + 2]
                            buf = buf[end + 2 :]
                            arr = np.frombuffer(jpg, dtype=np.uint8)
                            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                            if img is not None:
                                with self.lock:
                                    self.frame = img
                                    self.ts = time.time()
                            start, end = buf.find(b"\xff\xd8"), buf.find(b"\xff\xd9")
            except Exception as exc:  # noqa: BLE001
                print(f"[cam {self.url}] {exc!r}", flush=True)
                time.sleep(1.0)

    def get(self) -> tuple[np.ndarray | None, float]:
        with self.lock:
            if self.frame is None:
                return None, 0.0
            return self.frame.copy(), self.ts


def _joint_loop(path: Path, host: str, port: int, stop: threading.Event) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        while not stop.is_set():
            try:
                with socket.create_connection((host, port), timeout=3.0) as sock:
                    rf = sock.makefile("r", encoding="utf-8", newline="\n")
                    print(f"[joints] {host}:{port}", flush=True)
                    for line in rf:
                        if stop.is_set():
                            break
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            payload = json.loads(line)
                        except Exception:
                            continue
                        row = {"t": time.time(), "name": payload.get("name"), "position": payload.get("position")}
                        f.write(json.dumps(row) + "\n")
                        f.flush()
            except Exception as exc:  # noqa: BLE001
                print(f"[joints] reconnect: {exc!r}", flush=True)
                time.sleep(1.0)


def _action_loop(path: Path, port: int, stop: threading.Event) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(8)
    srv.settimeout(1.0)
    print(f"[session] listening 0.0.0.0:{port}", flush=True)
    with path.open("a", encoding="utf-8") as f:
        while not stop.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except Exception:
                break
            with conn:
                conn.settimeout(2.0)
                data = b""
                try:
                    while True:
                        chunk = conn.recv(65536)
                        if not chunk:
                            break
                        data += chunk
                except Exception:
                    pass
            for line in data.decode("utf-8", errors="replace").splitlines():
                line = line.strip()
                if not line:
                    continue
                f.write(line + "\n")
            f.flush()
    srv.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--http-cam", default="http://10.229.66.95:8766")
    parser.add_argument("--tcp-joints", default="10.229.66.95:8765")
    parser.add_argument("--session-port", type=int, default=8777)
    parser.add_argument(
        "--out-dir",
        default=os.environ.get("R1LITE_SESSION_ROOT", "/home/yao/r1lite_lerobot_datasets/sessions"),
    )
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl+C")
    parser.add_argument("--scale", type=float, default=1.0)
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = Path(args.out_dir) / f"session_{stamp}"
    (out / "videos").mkdir(parents=True, exist_ok=True)
    (out / "meta").mkdir(parents=True, exist_ok=True)

    readers = {n: MjpegReader(args.http_cam, n) for n in CAMERAS}
    for r in readers.values():
        r.start()

    stop = threading.Event()
    jhost, jport = args.tcp_joints.rsplit(":", 1)
    threading.Thread(
        target=_joint_loop, args=(out / "meta" / "joints.jsonl", jhost, int(jport), stop), daemon=True
    ).start()
    threading.Thread(
        target=_action_loop, args=(out / "meta" / "actions.jsonl", args.session_port, stop), daemon=True
    ).start()

    # wait first frames
    deadline = time.time() + 20.0
    sizes = {}
    while time.time() < deadline:
        ok = True
        for n, r in readers.items():
            fr, _ = r.get()
            if fr is None:
                ok = False
            else:
                sizes[n] = (fr.shape[1], fr.shape[0])
        if ok:
            break
        time.sleep(0.2)
    else:
        raise SystemExit("camera frames not ready")

    writers = {}
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    for n in CAMERAS:
        w, h = sizes[n]
        if args.scale != 1.0:
            w, h = int(w * args.scale), int(h * args.scale)
        path = out / "videos" / f"{n}.mp4"
        writers[n] = cv2.VideoWriter(str(path), fourcc, args.fps, (w, h))
        print(f"[video] {path} {w}x{h}@{args.fps}", flush=True)

    info = {
        "started": stamp,
        "http_cam": args.http_cam,
        "tcp_joints": args.tcp_joints,
        "session_port": args.session_port,
        "fps": args.fps,
        "cameras": sizes,
        "out": str(out),
    }
    (out / "meta" / "info.json").write_text(json.dumps(info, indent=2))

    period = 1.0 / max(args.fps, 1.0)
    t0 = time.time()
    nframes = 0
    print(f"[record] writing to {out}  (Ctrl+C to stop)", flush=True)
    try:
        while True:
            if args.seconds > 0 and (time.time() - t0) >= args.seconds:
                break
            loop_t = time.perf_counter()
            for n, r in readers.items():
                fr, _ = r.get()
                if fr is None:
                    continue
                if args.scale != 1.0:
                    fr = cv2.resize(fr, None, fx=args.scale, fy=args.scale)
                writers[n].write(fr)
            nframes += 1
            if nframes % int(max(args.fps, 1)) == 0:
                print(f"[record] frames={nframes} elapsed={time.time()-t0:.1f}s", flush=True)
            elapsed = time.perf_counter() - loop_t
            time.sleep(max(0.0, period - elapsed))
    except KeyboardInterrupt:
        print("[record] interrupted", flush=True)
    finally:
        stop.set()
        for w in writers.values():
            w.release()
        for r in readers.values():
            r.stop()
        info["frames"] = nframes
        info["elapsed_s"] = time.time() - t0
        (out / "meta" / "info.json").write_text(json.dumps(info, indent=2))
        print(f"[record] done frames={nframes} -> {out}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
