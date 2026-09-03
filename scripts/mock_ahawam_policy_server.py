#!/usr/bin/env python3
"""Minimal mock AHA-WAM policy server for offline bring-up (hold-pose chunk)."""

from __future__ import annotations

import argparse
import socket
import threading
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from deploy.tcp_protocol import recv_msg, send_msg  # noqa: E402


def handle(conn: socket.socket, step: list[int]) -> None:
    try:
        conn.settimeout(120.0)
        req = recv_msg(conn)
        if not isinstance(req, dict) or req.get("type") != "infer":
            send_msg(conn, {"ok": False, "error": "bad request"})
            return
        state = np.asarray(req["state"], dtype=np.float32).reshape(-1)
        if state.shape[0] != 14:
            send_msg(conn, {"ok": False, "error": f"state dim {state.shape}"})
            return
        chunk = np.tile(state[None, :], (16, 1)).astype(np.float32)
        # tiny drift so clients see non-identical rows
        chunk += (np.arange(16, dtype=np.float32)[:, None] * 1e-4)
        step[0] += 1
        send_msg(
            conn,
            {
                "ok": True,
                "type": "action_chunk",
                "action_chunk": chunk,
                "model_latency_ms": 1.0,
                "server_inference_step": step[0],
            },
        )
    except Exception as exc:  # noqa: BLE001
        try:
            send_msg(conn, {"ok": False, "error": repr(exc)})
        except Exception:
            pass
    finally:
        conn.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=10000)
    args = parser.parse_args()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(16)
    print(f"mock policy on {args.host}:{args.port}", flush=True)
    step = [0]
    while True:
        conn, addr = srv.accept()
        print(f"client {addr}", flush=True)
        threading.Thread(target=handle, args=(conn, step), daemon=True).start()


if __name__ == "__main__":
    raise SystemExit(main())
