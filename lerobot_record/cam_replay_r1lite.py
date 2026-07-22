#!/usr/bin/env python3
"""Replay R1Lite triple-camera episode videos (PyAV). Independent of Franka cam_replay."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import av
import cv2
import numpy as np

CAMERAS = ("head", "left_wrist", "right_wrist")
BAR_H = 56
BTN_W = 120
BTN_H = 40


def _latest_video(root: Path, cam_key: str) -> Path | None:
    base = root / "videos" / f"observation.images.{cam_key}"
    if not base.is_dir():
        return None
    files = sorted(base.rglob("*.mp4"), key=lambda p: p.stat().st_mtime)
    return files[-1] if files else None


def _decode_all(path: Path) -> list[np.ndarray]:
    frames: list[np.ndarray] = []
    with av.open(str(path)) as container:
        stream = container.streams.video[0]
        stream.thread_type = "AUTO"
        for packet in container.demux(stream):
            for frame in packet.decode():
                frames.append(frame.to_ndarray(format="bgr24"))
    return frames


def _window_closed(win: str) -> bool:
    try:
        prop = cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE)
        return prop < 1
    except Exception:  # noqa: BLE001
        return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--scale", type=float, default=0.75)
    args = parser.parse_args()
    root = Path(args.root)

    paths = {n: _latest_video(root, n) for n in CAMERAS}
    missing = [n for n, p in paths.items() if p is None]
    if missing:
        raise SystemExit(f"missing videos for: {missing} under {root}")

    print("decoding...", flush=True)
    decoded = {n: _decode_all(p) for n, p in paths.items()}  # type: ignore[arg-type]
    n_frames = min(len(decoded[n]) for n in CAMERAS)
    if n_frames <= 0:
        raise SystemExit("empty videos")

    info = {}
    info_path = root / "meta" / "info.json"
    if info_path.is_file():
        info = json.loads(info_path.read_text())
    fps = float(info.get("fps") or 15)
    delay = max(1, int(1000 / fps))

    win = "R1Lite Replay | head + left_wrist + right_wrist"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    idx = 0
    paused = False
    click = {"close": False}

    def on_mouse(event, x, y, _flags, _param) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        h = getattr(on_mouse, "h", 0)
        w = getattr(on_mouse, "w", 0)
        if h <= 0 or y < h - BAR_H:
            return
        # CLOSE button left
        if 16 <= x <= 16 + BTN_W and (h - BAR_H + 8) <= y <= (h - BAR_H + 8 + BTN_H):
            click["close"] = True

    cv2.setMouseCallback(win, on_mouse)

    while True:
        panels = []
        for name in CAMERAS:
            panel = decoded[name][idx].copy()
            cv2.putText(
                panel,
                f"{name} {idx+1}/{n_frames}",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
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
        cv2.rectangle(bar, (16, 8), (16 + BTN_W, 8 + BTN_H), (0, 0, 160), -1)
        cv2.putText(bar, "CLOSE", (36, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        cv2.putText(
            bar,
            f"{'PAUSE' if paused else 'PLAY'}  space/a/d/r/q",
            (160, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (200, 200, 200),
            1,
        )
        out = np.vstack([canvas, bar])
        on_mouse.h = out.shape[0]
        on_mouse.w = out.shape[1]
        cv2.imshow(win, out)

        if click["close"] or _window_closed(win):
            break
        key = cv2.waitKey(delay if not paused else 30) & 0xFF
        if key in (27, ord("q")) or click["close"]:
            break
        if key == ord(" "):
            paused = not paused
        elif key == ord("a"):
            idx = max(0, idx - 5)
        elif key == ord("d"):
            idx = min(n_frames - 1, idx + 5)
        elif key == ord("r"):
            idx = 0
        elif not paused:
            idx += 1
            if idx >= n_frames:
                if args.loop:
                    idx = 0
                else:
                    break

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
