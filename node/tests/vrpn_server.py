"""A VRPN tracker server stand-in: cookie exchange, sender and type descriptions, then a stream of poses."""

from __future__ import annotations

import socket
import struct
import threading
import time

POSE_TYPE = 7


def message(kind: int, sender: int, payload: bytes, stamp: float = 0.0) -> bytes:
    sec = int(stamp)
    usec = int(round((stamp - sec) * 1e6))
    header = struct.pack(">IIIii", 24 + len(payload), sec, usec, sender, kind) + b"\0" * 4
    return header + payload + b"\0" * (-len(payload) % 8)


def description(kind: int, ident: int, name: str) -> bytes:
    text = name.encode() + b"\0"
    return message(kind, ident, struct.pack(">I", len(text)) + text)


def pose(sender: int, position, quaternion=(0.0, 0.0, 0.0, 1.0), stamp: float = 0.0, kind: int = POSE_TYPE) -> bytes:
    return message(kind, sender, struct.pack(">ii7d", 0, 0, *position, *quaternion), stamp)


class FakeVrpn:
    def __init__(self, names=("Chaser", "Target"), rate_hz: float = 200.0) -> None:
        self.names = list(names)
        self.positions = {name: [0.1 * i, 0.2 * i, 0.3 * i] for i, name in enumerate(self.names)}
        self.rate_hz = rate_hz
        self.frozen = False
        self.silent = False
        self.connections = 0
        self.listener = socket.create_server(("127.0.0.1", 0))
        self.port = self.listener.getsockname()[1]
        self._client: socket.socket | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def __enter__(self) -> FakeVrpn:
        self._thread.start()
        return self

    def __exit__(self, *_) -> None:
        self._stop.set()
        self.drop()
        self.listener.close()
        self._thread.join(5)

    def drop(self) -> None:
        client, self._client = self._client, None
        if client is not None:
            try:
                client.close()
            except OSError:
                pass

    def _serve(self) -> None:
        self.listener.settimeout(0.1)
        while not self._stop.is_set():
            try:
                client, _ = self.listener.accept()
            except OSError:
                continue
            self.connections += 1
            self._client = client
            try:
                self._session(client)
            except OSError:
                pass

    def _session(self, client: socket.socket) -> None:
        client.settimeout(2)
        cookie = b""
        while len(cookie) < 24:
            chunk = client.recv(24 - len(cookie))
            if not chunk:
                return
            cookie += chunk
        client.sendall(b"vrpn: ver. 07.35  0".ljust(24, b"\0"))
        intro = b"".join(description(-1, i, name) for i, name in enumerate(self.names))
        client.sendall(intro + description(-2, POSE_TYPE, "vrpn_Tracker Pos_Quat"))
        tick = 0
        while not self._stop.is_set() and self._client is client:
            time.sleep(1.0 / self.rate_hz)
            if self.silent:
                continue
            tick += 1
            stamp = time.time()
            frames = b""
            for i, name in enumerate(self.names):
                jitter = 0.0 if self.frozen else 1e-6 * (tick % 7)
                frames += pose(i, [v + jitter for v in self.positions[name]], stamp=stamp)
            client.sendall(frames)
