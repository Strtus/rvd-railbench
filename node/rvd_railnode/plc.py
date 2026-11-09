"""Modbus TCP driver for PLC programs that expose each axis as target, speed and encoder registers."""

from __future__ import annotations

import math
import socket
import struct
import threading
import time
from typing import Any, Callable, Iterable

from .backend import BackendError, InterlockError, Sample
from .profile import axes as profile_axes


class ModbusError(BackendError):
    pass


class Modbus:
    """One Modbus TCP connection. A failed exchange closes the socket and the next call reconnects."""

    def __init__(self, host: str, port: int = 502, unit: int = 1, timeout_s: float = 0.25, *, d_base: int,
                 m_base: int, d_stride: int = 2, word_order: str = "low_first", allow_writes: bool = False,
                 retry_s: float = 0.5, connect: Callable[..., Any] = socket.create_connection,
                 clock: Callable[[], float] = time.monotonic, sleep: Callable[[float], None] = time.sleep) -> None:
        self.host, self.port, self.unit, self.timeout_s = host, port, unit, timeout_s
        self.d_base, self.d_stride, self.m_base = d_base, d_stride, m_base
        self.low_first = word_order == "low_first"
        self.allow_writes = allow_writes
        self.retry_s = retry_s
        self._connect, self._clock, self._sleep = connect, clock, sleep
        self._sock: Any = None
        self._tid = 0
        self._next_attempt = -math.inf
        self.generation = 0
        self.lock = threading.RLock()

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def ensure(self, retry_now: bool = False) -> None:
        with self.lock:
            if self._sock is not None:
                return
            now = self._clock()
            if not retry_now and now < self._next_attempt:
                raise ModbusError(f"PLC {self.host}:{self.port} is unreachable")
            self._next_attempt = now + self.retry_s
            try:
                sock = self._connect((self.host, self.port), timeout=self.timeout_s)
                sock.settimeout(self.timeout_s)
            except OSError as exc:
                raise ModbusError(f"cannot connect to PLC {self.host}:{self.port}: {exc}") from None
            self._sock = sock
            self.generation += 1

    def close(self) -> None:
        with self.lock:
            sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def _recv(self, n: int) -> bytes:
        data = b""
        while len(data) < n:
            part = self._sock.recv(n - len(data))
            if not part:
                raise ModbusError("PLC closed the connection")
            data += part
        return data

    def transact(self, pdu: bytes) -> bytes:
        if pdu[0] != 3 and not self.allow_writes:
            raise ModbusError("PLC writes are disabled on a read-only node")
        with self.lock:
            self.ensure()
            self._tid = (self._tid + 1) & 0xFFFF
            try:
                self._sock.sendall(struct.pack(">HHHB", self._tid, 0, len(pdu) + 1, self.unit) + pdu)
                tid, proto, size, unit = struct.unpack(">HHHB", self._recv(7))
                if tid != self._tid or proto != 0 or unit != self.unit or not 2 <= size <= 254:
                    raise ModbusError("invalid PLC response header")
                reply = self._recv(size - 1)
            except (OSError, ModbusError) as exc:
                self.close()
                raise ModbusError(f"PLC I/O failed: {exc}") from None
        if reply[0] != pdu[0]:
            raise ModbusError(f"PLC rejected function {pdu[0]}: {reply.hex()}")
        return reply

    def _d_address(self, d: int) -> int:
        address = self.d_base + self.d_stride * d
        if not 0 <= address <= 0xFFFE:
            raise ModbusError(f"D{d} is outside the Modbus address space")
        return address

    def read_word(self, address: int) -> int:
        reply = self.transact(struct.pack(">BHH", 3, address, 1))
        if len(reply) != 4 or reply[1] != 2:
            raise ModbusError("invalid register reply")
        return struct.unpack(">H", reply[2:])[0]

    def read_d(self, d: int) -> int:
        reply = self.transact(struct.pack(">BHH", 3, self._d_address(d), 2))
        if len(reply) != 6 or reply[1] != 4:
            raise ModbusError("invalid D register reply")
        first, second = struct.unpack(">HH", reply[2:])
        lo, hi = (first, second) if self.low_first else (second, first)
        value = lo | hi << 16
        return value - (1 << 32) if value & 0x80000000 else value

    def write_d(self, d: int, value: int) -> None:
        if not -(1 << 31) <= value < 1 << 31:
            raise ModbusError(f"D{d} value {value} does not fit in 32 bits")
        address = self._d_address(d)
        raw = value & 0xFFFFFFFF
        words = (raw & 0xFFFF, raw >> 16) if self.low_first else (raw >> 16, raw & 0xFFFF)
        if self.transact(struct.pack(">BHHBHH", 16, address, 2, 4, *words)) != struct.pack(">BHH", 16, address, 2):
            raise ModbusError(f"D{d} write was not acknowledged")

    def read_m(self, m: int) -> bool:
        return bool(self.read_word(self.m_base + m // 16) >> (m % 16) & 1)

    def write_m(self, m: int, value: bool) -> None:
        # M bits share 16-bit words (read-modify-write); the node must be the only PC-side writer.
        with self.lock:
            address, mask = self.m_base + m // 16, 1 << (m % 16)
            before = self.read_word(address)
            after = before | mask if value else before & ~mask
            if after != before:
                pdu = struct.pack(">BHH", 6, address, after)
                if self.transact(pdu) != pdu:
                    raise ModbusError(f"M{m} write was not acknowledged")

    def pulse_m(self, m: int, low_s: float = 0.03) -> None:
        self.write_m(m, False)
        self._sleep(low_s)  # let the PLC scan the low level before the rising edge
        self.write_m(m, True)


class PlcBackend:
    name = "modbus"

    def __init__(self, profile: dict[str, Any], allow_motion: bool = False, *, io: Any = None,
                 clock: Callable[[], float] = time.monotonic, sleep: Callable[[float], None] = time.sleep) -> None:
        self.profile = profile
        self.axes = profile_axes(profile)
        self.allow_motion = allow_motion
        self._spec = {n: profile["axes"][n] for n in self.axes}
        bus = profile["modbus"]
        self.io = io or Modbus(bus["host"], bus.get("port", 502), bus.get("unit", 1), bus.get("timeout_s", 0.25),
                               d_base=bus["d_base"], m_base=bus["m_base"], d_stride=bus.get("d_stride", 2),
                               word_order=bus.get("word_order", "low_first"), allow_writes=allow_motion,
                               clock=clock, sleep=sleep)
        self._interlock = profile.get("interlock", {})
        self._clock, self._sleep = clock, sleep
        self._verified = -1
        self.armed: set[str] = set()
        self.active = False
        self.raw: dict[str, int] = {}
        self.status: dict[str, Any] = {}

    @property
    def connected(self) -> bool:
        return self.io.connected

    def _check_interlock(self, identity: bool | None = None) -> None:
        self.io.ensure()
        expected = self._interlock.get("identity")
        if expected and (identity or self.io.generation != self._verified):
            value = self.io.read_d(expected["d"])
            self.status["identity"] = value
            if value != expected["value"]:
                raise InterlockError(f"PLC program D{expected['d']}={value}, the profile expects {expected['value']}")
            self._verified = self.io.generation
        if "emergency_m" in self._interlock:
            flag = self.io.read_m(self._interlock["emergency_m"])
            self.status["emergency"] = flag
            if flag:
                raise InterlockError(f"PLC emergency flag M{self._interlock['emergency_m']} is set")

    def _position(self, name: str, count: int) -> float:
        spec = self._spec[name]
        return (count - spec["zero"]) * spec["scale"]

    def _count(self, name: str, value: float) -> int:
        spec = self._spec[name]
        count = round(value / spec["scale"] + spec["zero"])
        if not spec["min_count"] <= count <= spec["max_count"]:
            raise ModbusError(f"{name} target {value:g} is count {count}, outside "
                              f"[{spec['min_count']}, {spec['max_count']}]")
        return count

    def _speed_count(self, name: str, speed: float) -> int:
        limit = self.axes[name].max_speed
        if not math.isfinite(speed) or not 0 < speed <= limit * (1 + 1e-9):
            raise ModbusError(f"{name} speed {speed:g} is outside (0, {limit:g}]")
        return max(1, round(speed / abs(self._spec[name]["scale"])))

    def _require_motion(self) -> None:
        if not self.allow_motion:
            raise ModbusError("node is read-only; start it with --allow-motion")

    def _clear_stop(self, spec: dict[str, Any]) -> None:
        self.io.write_m(spec["stop_m"], False)
        if spec.get("quickstop_m", spec["stop_m"]) != spec["stop_m"]:
            self.io.write_m(spec["quickstop_m"], False)

    def read(self) -> Sample:
        begin = self._clock()
        self._check_interlock()
        raw = {}
        for name, spec in self._spec.items():
            if "sample_count_d" in spec and self.io.read_d(spec["sample_count_d"]) == 0:
                raise ModbusError(f"{name} encoder reports no valid samples")
            raw[name] = self.io.read_d(spec["encoder_d"])
        for name in sorted(self.armed):
            spec = self._spec[name]
            if spec["type"] == "servo" and self.io.read_d(spec["enabled_d"]) != 1:
                raise ModbusError(f"{name} drive is no longer enabled")
        self.raw = raw
        return Sample({n: self._position(n, c) for n, c in raw.items()}, begin, self._clock() - begin, dict(raw))

    def arm(self, names: Iterable[str]) -> None:
        self._require_motion()
        names = [n for n in names if n not in self.armed]
        self._check_interlock(identity=True)
        for name in names:
            for check in self._spec[name].get("ready", []):
                value = self.io.read_d(check["d"])
                if value != check["value"]:
                    raise ModbusError(f"{name} is not ready: D{check['d']}={value}, expected {check['value']}")
        self.active = True  # any failure from here on must be followed by a stop
        for name in names:
            spec = self._spec[name]
            if spec["type"] == "servo":
                self.io.write_d(spec["target_d"], self.io.read_d(spec["encoder_d"]))
                self.io.write_d(spec["speed_d"], self._speed_count(name, 0.1 * self.axes[name].max_speed))
                self._clear_stop(spec)
                self.io.pulse_m(spec["enable_m"])
                deadline = self._clock() + self.profile.get("enable_timeout_s", 2.0)
                while self.io.read_d(spec["enabled_d"]) != 1:
                    if self._clock() >= deadline:
                        raise ModbusError(f"{name} enable was not confirmed")
                    self._sleep(0.02)
            self.armed.add(name)

    def command(self, targets: dict[str, float], speeds: dict[str, float]) -> None:
        self._require_motion()
        self._check_interlock()
        staged = []
        for name, value in targets.items():
            if name not in self.armed:
                raise ModbusError(f"{name} is not armed")
            staged.append((self._spec[name], self._count(name, value), self._speed_count(name, speeds[name])))
        self.active = True
        for spec, count, speed in staged:
            self.io.write_d(spec["target_d"], count)
            self.io.write_d(spec["speed_d"], speed)
        for spec, _, _ in staged:
            if spec["type"] == "pulse":
                self._clear_stop(spec)
                self.io.pulse_m(spec["start_m"])  # a new target needs a new start edge

    def _read_retrying(self, d: int) -> int:
        try:
            return self.io.read_d(d)
        except ModbusError:
            self.io.ensure(retry_now=True)
            return self.io.read_d(d)

    def _confirm_stopped(self) -> dict[str, int]:
        deadline = self._clock() + self.profile.get("stop_timeout_s", 5.0)
        last = {n: self._read_retrying(s["encoder_d"]) for n, s in self._spec.items()}
        stable = dict.fromkeys(self._spec, 0)
        while True:
            self._sleep(0.1)
            done = True
            for name, spec in self._spec.items():
                count = self._read_retrying(spec["encoder_d"])
                stable[name] = stable[name] + 1 if abs(count - last[name]) <= 2 else 0
                last[name] = count
                ok = stable[name] >= 3
                if ok and spec["type"] == "servo":
                    ok = self._read_retrying(spec["enabled_d"]) == 0 and self._read_retrying(spec["closed_d"]) == 1
                    if ok and "pending_d" in spec:
                        ok = self._read_retrying(spec["pending_d"]) == 0
                done = done and ok
            if done:
                return last
            if self._clock() >= deadline:
                raise ModbusError("stop was sent but not every axis confirmed standstill")

    def stop(self, force: bool = False) -> None:
        if not self.allow_motion or not (self.active or force):
            return
        pending, error = list(self._spec), ""
        for attempt in range(3):
            if attempt:
                self._sleep(0.1 * attempt)
            try:
                self.io.ensure(retry_now=True)
            except ModbusError as exc:
                error = str(exc)
                continue
            for name in list(pending):
                spec = self._spec[name]
                try:
                    self.io.write_m(spec["start_m"] if spec["type"] == "pulse" else spec["enable_m"], False)
                    self.io.pulse_m(spec["stop_m"])
                    pending.remove(name)
                except ModbusError as exc:
                    error = f"{name}: {exc}"
            if not pending:
                break
        if pending:
            raise ModbusError(f"stop not delivered to {', '.join(pending)} ({error})")
        last = self._confirm_stopped()
        for name, spec in self._spec.items():
            self.io.write_d(spec["target_d"], last[name])
            self.io.write_d(spec["speed_d"], 0)
        self.armed.clear()
        self.active = False

    def close(self) -> None:
        try:
            self.stop()
        finally:
            self.io.close()
