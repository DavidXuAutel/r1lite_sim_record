#!/usr/bin/env python3
"""On R1Lite PC: forward :10000 -> AHA-WAM policy pod via SSH gateway.

Gateway publishes only SSH :32496, not policy :10000. Run this on the robot:

    python3 scripts/ahawam_policy_ssh_tunnel.py
    # then clients on robot use --server-ip 127.0.0.1 --server-port 10000

Does NOT go through Franka host 10.229.20.125.
"""

from __future__ import annotations

import argparse
import select
import socket
import threading
import time

import paramiko

DEFAULT_GATEWAY = ("10.239.121.25", 32496)
DEFAULT_USER = "a26413"
DEFAULT_REMOTE = ("127.0.0.1", 10000)


def pipe(src: socket.socket, dst) -> None:
    try:
        while True:
            data = src.recv(65536)
            if not data:
                break
            dst.sendall(data)
    except Exception:
        pass
    finally:
        for x in (src, dst):
            try:
                x.close()
            except Exception:
                pass


def handle(client: socket.socket, transport: paramiko.Transport, remote: tuple[str, int]) -> None:
    try:
        chan = transport.open_channel("direct-tcpip", remote, client.getpeername())
    except Exception as exc:  # noqa: BLE001
        print("open_channel failed:", exc, flush=True)
        client.close()
        return
    threading.Thread(target=pipe, args=(client, chan), daemon=True).start()
    threading.Thread(target=pipe, args=(chan, client), daemon=True).start()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--gateway-host", default=DEFAULT_GATEWAY[0])
    p.add_argument("--gateway-port", type=int, default=DEFAULT_GATEWAY[1])
    p.add_argument("--user", default=DEFAULT_USER)
    p.add_argument("--password", default="123456")
    p.add_argument("--listen-host", default="127.0.0.1", help="use 0.0.0.0 only if needed")
    p.add_argument("--listen-port", type=int, default=10000)
    p.add_argument("--remote-host", default=DEFAULT_REMOTE[0])
    p.add_argument("--remote-port", type=int, default=DEFAULT_REMOTE[1])
    args = p.parse_args()

    listen = (args.listen_host, args.listen_port)
    remote = (args.remote_host, args.remote_port)
    gateway = (args.gateway_host, args.gateway_port)

    while True:
        client = None
        srv = None
        try:
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect(
                gateway[0],
                port=gateway[1],
                username=args.user,
                password=args.password,
                allow_agent=False,
                look_for_keys=False,
                timeout=20,
            )
            transport = client.get_transport()
            assert transport is not None
            transport.set_keepalive(30)
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(listen)
            srv.listen(50)
            print(f"R1 tunnel {listen} -> {gateway} -> {remote}", flush=True)
            while transport.is_active():
                r, _, _ = select.select([srv], [], [], 1.0)
                if srv in r:
                    conn, addr = srv.accept()
                    print("client", addr, flush=True)
                    threading.Thread(
                        target=handle, args=(conn, transport, remote), daemon=True
                    ).start()
        except Exception as exc:  # noqa: BLE001
            print("tunnel restart:", exc, flush=True)
            time.sleep(2)
        finally:
            if srv is not None:
                try:
                    srv.close()
                except Exception:
                    pass
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass


if __name__ == "__main__":
    raise SystemExit(main())
