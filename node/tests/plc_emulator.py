"""A Modbus TCP server that behaves like a positioning PLC described by a profile."""

from __future__ import annotations

import socketserver
import struct
import threading
import time
from typing import Any


class _Axis:
    def __init__(self, spec: dict[str, Any], start: int) -> None:
        self.spec = spec
        self.position = float(start)
        self.target = float(start)
        self.speed = 0.0
        self.moving = False
        self.enabled = False


class PlcEmulator:
    def __init__(self, profile: dict[str, Any], start: dict[str, int] | None = None, tick_s: float = 0.005) -> None:
        bus = profile["modbus"]
        self.d_base, self.d_stride, self.m_base = bus["d_base"], bus.get("d_stride", 2), bus["m_base"]
        self.low_first = bus.get("word_order", "low_first") == "low_first"
        self.words: dict[int, int] = {}
        self.lock = threading.Lock()
        self.requests = 0
        self.tick_s = tick_s
        self.axes = {}
        for name, spec in profile["axes"].items():
            begin = (start or {}).get(name, (spec["min_count"] + spec["max_count"]) // 2)
            self.axes[name] = _Axis(spec, begin)
            self.set_d(spec["target_d"], begin)
            if "sample_count_d" in spec:
                self.set_d(spec["sample_count_d"], 1)
            for check in spec.get("ready", []):
                self.set_d(check["d"], check["value"])
            if spec["type"] == "servo":
                self.set_d(spec["closed_d"], 1)
        identity = profile.get("interlock", {}).get("identity")
        if identity:
            self.set_d(identity["d"], identity["value"])
        bits = [v for spec in profile["axes"].values() for k, v in spec.items() if k.endswith("_m")]
        bits.append(profile.get("interlock", {}).get("emergency_m", 0))
        self.m_end = self.m_base + max(bits) // 16
        self._publish()
        emulator = self

        class Handler(socketserver.BaseRequestHandler):
            def handle(self) -> None:
                emulator._serve(self.request)

        class Server(socketserver.ThreadingTCPServer):
            daemon_threads = True
            allow_reuse_address = True

        self.server = Server(("127.0.0.1", 0), Handler)
        self.port = self.server.server_address[1]
        self._stop = threading.Event()
        self._threads = [threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": 0.05},
                                          daemon=True), threading.Thread(target=self._physics, daemon=True)]

    def __enter__(self) -> PlcEmulator:
        for thread in self._threads:
            thread.start()
        return self

    def __exit__(self, *_: Any) -> None:
        self._stop.set()
        self.server.shutdown()
        self.server.server_close()

    # register image

    def d_address(self, d: int) -> int:
        return self.d_base + self.d_stride * d

    def set_d(self, d: int, value: int) -> None:
        raw = value & 0xFFFFFFFF
        lo, hi = raw & 0xFFFF, raw >> 16
        address = self.d_address(d)
        self.words[address], self.words[address + 1] = (lo, hi) if self.low_first else (hi, lo)

    def get_d(self, d: int) -> int:
        address = self.d_address(d)
        first, second = self.words.get(address, 0), self.words.get(address + 1, 0)
        lo, hi = (first, second) if self.low_first else (second, first)
        value = lo | hi << 16
        return value - (1 << 32) if value & 0x80000000 else value

    def m(self, bit: int) -> bool:
        return bool(self.words.get(self.m_base + bit // 16, 0) >> (bit % 16) & 1)

    def set_m(self, bit: int, value: bool) -> None:
        with self.lock:
            address, mask = self.m_base + bit // 16, 1 << (bit % 16)
            word = self.words.get(address, 0)
            self.words[address] = word | mask if value else word & ~mask

    # behaviour

    def _edges(self, address: int, before: int, after: int) -> None:
        if not self.m_base <= address <= self.m_end:
            return
        for bit in range(16):
            mask = 1 << bit
            m = (address - self.m_base) * 16 + bit
            rose, fell = after & mask and not before & mask, before & mask and not after & mask
            for axis in self.axes.values():
                spec = axis.spec
                if spec["type"] == "pulse" and rose and m == spec["start_m"]:
                    axis.target, axis.speed = float(self.get_d(spec["target_d"])), float(self.get_d(spec["speed_d"]))
                    axis.moving = True
                elif spec["type"] == "servo" and rose and m == spec["enable_m"]:
                    axis.enabled = True
                    axis.target = float(self.get_d(spec["target_d"]))
                elif spec["type"] == "servo" and fell and m == spec["enable_m"]:
                    axis.enabled = False
                if rose and m == spec["stop_m"]:
                    axis.moving, axis.enabled = False, False
                    axis.target = axis.position

    def _publish(self) -> None:
        for axis in self.axes.values():
            spec = axis.spec
            self.set_d(spec["encoder_d"], round(axis.position))
            if spec["type"] == "servo":
                self.set_d(spec["enabled_d"], 1 if axis.enabled else 0)
                self.set_d(spec["closed_d"], 0 if axis.enabled else 1)
                if "pending_d" in spec:
                    self.set_d(spec["pending_d"], 0)

    def _physics(self) -> None:
        last = time.monotonic()
        while not self._stop.wait(self.tick_s):
            now = time.monotonic()
            dt, last = now - last, now
            with self.lock:
                for axis in self.axes.values():
                    spec = axis.spec
                    if spec["type"] == "servo" and axis.enabled:
                        axis.target = float(self.get_d(spec["target_d"]))
                        axis.speed = float(self.get_d(spec["speed_d"]))
                        axis.moving = True
                    if axis.moving:
                        step = axis.speed * dt
                        gap = axis.target - axis.position
                        if abs(gap) <= step:
                            axis.position = axis.target
                            axis.moving = spec["type"] == "servo" and axis.enabled
                        else:
                            axis.position += step if gap > 0 else -step
                self._publish()

    # Modbus TCP

    def _serve(self, sock: Any) -> None:
        while True:
            header = self._recv(sock, 7)
            if header is None:
                return
            tid, _, size, unit = struct.unpack(">HHHB", header)
            pdu = self._recv(sock, size - 1)
            if pdu is None:
                return
            with self.lock:
                self.requests += 1
                reply = self._answer(pdu)
            sock.sendall(struct.pack(">HHHB", tid, 0, len(reply) + 1, unit) + reply)

    @staticmethod
    def _recv(sock: Any, n: int) -> bytes | None:
        data = b""
        while len(data) < n:
            try:
                part = sock.recv(n - len(data))
            except OSError:
                return None
            if not part:
                return None
            data += part
        return data

    def _answer(self, pdu: bytes) -> bytes:
        function = pdu[0]
        if function == 3:
            address, count = struct.unpack(">HH", pdu[1:5])
            words = [self.words.get(address + i, 0) for i in range(count)]
            return bytes([3, 2 * count]) + struct.pack(f">{count}H", *words)
        if function == 6:
            address, value = struct.unpack(">HH", pdu[1:5])
            before = self.words.get(address, 0)
            self.words[address] = value
            self._edges(address, before, value)
            return pdu[:5]
        if function == 16:
            address, count, _ = struct.unpack(">HHB", pdu[1:6])
            values = struct.unpack(f">{count}H", pdu[6:6 + 2 * count])
            for i, value in enumerate(values):
                before = self.words.get(address + i, 0)
                self.words[address + i] = value
                self._edges(address + i, before, value)
            return pdu[:5]
        return bytes([function | 0x80, 1])
