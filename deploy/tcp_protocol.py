#!/usr/bin/env python3
"""TCP length-prefix + pickle framing (AHA-WAM deploy/common/tcp_protocol compatible)."""

from __future__ import annotations

import pickle
import socket
import struct
from typing import Any


def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed while receiving")
        buf += chunk
    return buf


def send_msg(sock: socket.socket, obj: Any) -> None:
    payload = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
    sock.sendall(struct.pack(">I", len(payload)) + payload)


def recv_msg(sock: socket.socket) -> Any:
    (length,) = struct.unpack(">I", recv_exact(sock, 4))
    if length > 64 * 1024 * 1024:
        raise ValueError(f"refusing oversized payload: {length}")
    return pickle.loads(recv_exact(sock, length))


class PolicyClient:
    def __init__(self, host: str, port: int, timeout_s: float = 120.0) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s

    def infer(self, instruction: str, state: Any, front_rgb) -> dict:
        req = {
            "type": "infer",
            "instruction": instruction,
            "state": state,
            "images": {"front": front_rgb},
        }
        with socket.create_connection((self.host, self.port), timeout=10.0) as sock:
            sock.settimeout(self.timeout_s)
            send_msg(sock, req)
            resp = recv_msg(sock)
        if not isinstance(resp, dict):
            raise RuntimeError(f"bad response type: {type(resp)}")
        return resp
