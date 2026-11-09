"""Motion capture over VRPN's TCP protocol (no compiled VRPN library): the latest pose of each named tracker."""

from __future__ import annotations

import math
import socket
import struct
import threading
import time
from typing import Any, Callable

COOKIE = b"vrpn: ver. 07.35  0".ljust(24, b"\0")
POSE_TYPE = "vrpn_Tracker Pos_Quat"
SENDER_DESCRIPTION, TYPE_DESCRIPTION = -1, -2
MAX_MESSAGE = 1 << 20

Pose = tuple[str, int, float, tuple[float, float, float], tuple[float, float, float, float]]


class VrpnError(ValueError):
    pass


def _name(raw: bytes) -> str:
    raw = raw.split(b"\0", 1)[0]
    try:
        return raw.decode("utf-8")
    except UnicodeDecodeError:
        return raw.decode("gb18030", "replace")


class Decoder:
    """Splits a VRPN TCP stream into messages; returns tracker poses (sender, sensor, time, position, quaternion)."""

    def __init__(self) -> None:
        self.buffer = b""
        self.senders: dict[int, str] = {}
        self.types: dict[int, str] = {}

    def feed(self, data: bytes) -> list[Pose]:
        self.buffer += data
        poses: list[Pose] = []
        while len(self.buffer) >= 24:
            length, sec, usec, sender, kind = struct.unpack(">IIIii", self.buffer[:20])
            if not 24 <= length <= MAX_MESSAGE:
                raise VrpnError(f"invalid VRPN message length {length}")
            total = 24 + (length - 24 + 7) // 8 * 8  # header and payload are padded to 8 bytes
            if len(self.buffer) < total:
                break
            payload, self.buffer = self.buffer[24:length], self.buffer[total:]
            if kind in (SENDER_DESCRIPTION, TYPE_DESCRIPTION):
                if len(payload) < 5:
                    raise VrpnError("short VRPN name description")
                size = struct.unpack(">I", payload[:4])[0]
                if not 1 <= size <= len(payload) - 4:
                    raise VrpnError("invalid VRPN name length")
                (self.senders if kind == SENDER_DESCRIPTION else self.types)[sender] = _name(payload[4:4 + size])
            elif kind >= 0 and self.types.get(kind) == POSE_TYPE:
                if len(payload) != 64:
                    raise VrpnError(f"tracker message of {len(payload)} bytes")
                sensor = struct.unpack(">i", payload[:4])[0]
                values = struct.unpack(">7d", payload[8:])
                if all(math.isfinite(v) for v in values):  # a lost body may be sent as NaN; it simply goes stale
                    poses.append((self.senders.get(sender, str(sender)), sensor, sec + usec * 1e-6,
                                  values[:3], values[3:]))  # type: ignore[arg-type]
        return poses


def _map(transform: dict[str, Any], p: tuple[float, float, float]) -> dict[str, float]:
    m, o = transform["matrix"], transform["offset"]
    return {axis: sum(m[i][j] * p[j] for j in range(3)) + o[i] for i, axis in enumerate("xyz")}


class MocapReceiver:
    """Keeps one VRPN connection alive in a thread and reports each tracker's latest pose and its health."""

    def __init__(self, config: dict[str, Any], clock: Callable[[], float] = time.monotonic,
                 connect: Callable[..., Any] = socket.create_connection) -> None:
        self.host, self.port = config["host"], config.get("port", 3883)
        self.local_ip = config.get("local_ip")
        self.trackers: dict[str, dict[str, Any]] = config["trackers"]
        self.max_age_s = config.get("max_age_s", 0.25)
        self.max_repeat_s = config.get("max_repeat_s", 0.5)
        self.silence_s = config.get("reconnect_after_s", 5.0)
        self._clock, self._connect = clock, connect
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sock: Any = None
        self._latest: dict[str, dict[str, Any]] = {}
        self.connected = False
        self.error: str | None = "not started"
        self.frames = 0
        self.connections = 0
        self._thread = threading.Thread(target=self._run, name="rail-node-mocap", daemon=True)

    def start(self) -> MocapReceiver:
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        sock = self._sock
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        if self._thread.is_alive():
            self._thread.join(5)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._session()
            except (OSError, VrpnError) as exc:
                with self._lock:
                    self.error = str(exc) or type(exc).__name__
            finally:
                with self._lock:
                    self.connected = False
                self._sock = None
            self._stop.wait(1.0)

    def _session(self) -> None:
        source = (self.local_ip, 0) if self.local_ip else None
        with self._connect((self.host, self.port), timeout=3.0, source_address=source) as sock:
            self._sock = sock
            sock.settimeout(0.5)
            sock.sendall(COOKIE)
            cookie, deadline = b"", self._clock() + 3.0
            while len(cookie) < 24:
                if self._stop.is_set():
                    return
                if self._clock() > deadline:
                    raise VrpnError("no VRPN cookie from the mocap server")
                try:
                    chunk = sock.recv(24 - len(cookie))
                except socket.timeout:
                    continue
                if not chunk:
                    raise VrpnError("mocap server closed the connection during the handshake")
                cookie += chunk
            if not cookie.startswith(b"vrpn: ver. 07."):
                raise VrpnError(f"not a VRPN 7 server: {cookie[:16]!r}")
            with self._lock:
                self.connected, self.error = True, None
                self.connections += 1
            decoder, heard = Decoder(), self._clock()
            while not self._stop.is_set():
                try:
                    data = sock.recv(65536)
                except socket.timeout:
                    if self._clock() - heard > self.silence_s:
                        raise VrpnError(f"no data from the mocap server for {self.silence_s:g} s") from None
                    continue
                if not data:
                    raise VrpnError("mocap server closed the connection")
                heard = self._clock()
                poses = decoder.feed(data)
                if poses:
                    self._store(poses, heard)

    def _store(self, poses: list[Pose], received: float) -> None:
        with self._lock:
            for name, sensor, stamp, position, quaternion in poses:
                spec = self.trackers.get(name)
                if spec is None or sensor != spec.get("sensor", 0):
                    continue
                last = self._latest.get(name)
                if last and stamp <= last["source_time"]:
                    continue  # duplicate or out of order
                same = last is not None and last["position"] == position and last["quaternion"] == quaternion
                self._latest[name] = {
                    "source_time": stamp, "received": received, "position": position, "quaternion": quaternion,
                    "unchanged_since": last["unchanged_since"] if same else received,  # type: ignore[index]
                    "frames": last["frames"] + 1 if last else 1,
                }
                self.frames += 1

    def state(self) -> dict[str, Any]:
        now = self._clock()
        with self._lock:
            trackers: dict[str, Any] = {}
            for name, spec in self.trackers.items():
                sample = self._latest.get(name)
                if sample is None:
                    trackers[name] = {"valid": False, "reason": "no pose received"}
                    continue
                age = now - sample["received"]
                unchanged = sample["received"] - sample["unchanged_since"]
                if not self.connected:
                    reason = "disconnected"
                elif age > self.max_age_s:
                    reason = f"no pose for {age:.2f} s"
                elif self.max_repeat_s is not None and unchanged > self.max_repeat_s:
                    reason = f"pose repeated bit for bit for {unchanged:.2f} s"  # a lost body replayed as frozen
                else:
                    reason = None
                entry = {"valid": reason is None, "reason": reason, "age_s": age, "received": sample["received"],
                         "source_time": sample["source_time"], "frames": sample["frames"],
                         "position_raw": list(sample["position"]), "quaternion_xyzw": list(sample["quaternion"])}
                if "to_rail" in spec:
                    entry["rail"] = _map(spec["to_rail"], sample["position"])
                trackers[name] = entry
            return {"source": f"vrpn://{self.host}:{self.port}", "connected": self.connected, "error": self.error,
                    "frames": self.frames, "connections": self.connections, "trackers": trackers}
