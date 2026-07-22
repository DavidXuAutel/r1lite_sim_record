#!/usr/bin/env python3
"""R1Lite camera relay: vendor topics -> /r1lite/cam/*/compressed + MJPEG :8766.

Naming:
  ROS node: /r1lite/camera_mjpeg
  ROS topics:
    /r1lite/cam/head/compressed
    /r1lite/cam/left_wrist/compressed
    /r1lite/cam/right_wrist/compressed
  HTTP: http://0.0.0.0:8766/stream/<name>
"""

from __future__ import annotations

import argparse
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

R1LITE_NS = "r1lite"
R1LITE_NODE = "camera_mjpeg"

VENDOR_TOPICS = {
    "head": "/hdas/camera_head/left_raw/image_raw_color/compressed",
    "left_wrist": "/hdas/camera_wrist_left/color/image_raw/compressed",
    "right_wrist": "/hdas/camera_wrist_right/color/image_raw/compressed",
}

PUB_TOPICS = {
    name: f"/r1lite/cam/{name}/compressed" for name in VENDOR_TOPICS
}


class CameraStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frames: dict[str, tuple[bytes, float, str]] = {}

    def set_jpeg(self, name: str, data: bytes, frame_id: str) -> None:
        with self._lock:
            self._frames[name] = (data, time.time(), frame_id)

    def get_jpeg(self, name: str) -> tuple[bytes, float, str] | None:
        with self._lock:
            return self._frames.get(name)


class CameraRelayNode(Node):
    def __init__(self, store: CameraStore) -> None:
        super().__init__(R1LITE_NODE, namespace=R1LITE_NS)
        self.store = store
        self._pubs = {
            name: self.create_publisher(CompressedImage, PUB_TOPICS[name], 10)
            for name in VENDOR_TOPICS
        }
        for name, topic in VENDOR_TOPICS.items():
            msg_type = CompressedImage if topic.endswith("/compressed") else Image
            self.create_subscription(
                msg_type,
                topic,
                lambda msg, camera_name=name: self._on_image(camera_name, msg),
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"sub {topic} -> pub {PUB_TOPICS[name]}")

    def _on_image(self, name: str, msg: CompressedImage | Image) -> None:
        try:
            if isinstance(msg, CompressedImage):
                data = bytes(msg.data)
                out = CompressedImage()
                out.header = msg.header
                out.format = msg.format or "jpeg"
                out.data = msg.data
            else:
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                frame = arr.reshape(msg.height, msg.width, -1)
                if frame.shape[2] >= 3:
                    frame = frame[:, :, :3]
                ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                if not ok:
                    return
                data = bytes(buf)
                out = CompressedImage()
                out.header = msg.header
                out.format = "jpeg"
                out.data = data
            self.store.set_jpeg(name, data, getattr(msg.header, "frame_id", "") or "")
            self._pubs[name].publish(out)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"failed {name}: {exc!r}")


def make_handler(store: CameraStore):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: Any) -> None:
            return

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path in {"/", "/index.html"}:
                body = (
                    "<html><body><h3>/r1lite/camera_mjpeg</h3>"
                    + "".join(
                        f'<div><b>{n}</b><br><img src="/stream/{n}" width="480"></div>'
                        for n in VENDOR_TOPICS
                    )
                    + "</body></html>"
                ).encode()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path.startswith("/stream/"):
                name = path.split("/", 2)[2]
                if name not in VENDOR_TOPICS:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                while True:
                    row = store.get_jpeg(name)
                    if row is None:
                        time.sleep(0.05)
                        continue
                    data, _ts, _fid = row
                    try:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode())
                        self.wfile.write(data)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                        time.sleep(0.05)
                    except (BrokenPipeError, ConnectionResetError):
                        return
            self.send_error(HTTPStatus.NOT_FOUND)

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8766)
    args = parser.parse_args()

    store = CameraStore()
    rclpy.init()
    node = CameraRelayNode(store)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(store))
    print(
        f"/{R1LITE_NS}/{R1LITE_NODE} MJPEG http://{args.host}:{args.port}/ "
        f"topics={list(PUB_TOPICS.values())}",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
