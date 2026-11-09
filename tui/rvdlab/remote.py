"""Runs on a rail node: the user's connection, manual moves, and the bridge that relays rail_main to the node."""

from __future__ import annotations

import os
import socket
import threading
import time
from pathlib import Path
from typing import Any, Callable

AXES = ("x", "y", "z")
# A run through the network and a PLC needs looser timing than a simulated one; rail_main still checks them.
REMOTE_TIMING = {"clock_skew_s": 0.1, "communication_timeout_s": 0.8, "measurement_late_s": 0.5}


class RemoteError(RuntimeError):
    pass


def connection_path(explicit: Path | None = None) -> Path | None:
    """--node, else $RVDLAB_NODE, else ~/.rvdlab/node/connection.json when present."""
    if explicit is not None:
        return Path(explicit).expanduser()
    if os.environ.get("RVDLAB_NODE"):
        return Path(os.environ["RVDLAB_NODE"]).expanduser()
    home = Path(os.environ.get("RVDLAB_HOME") or Path.home() / ".rvdlab").expanduser()
    path = home / "node" / "connection.json"
    return path if path.is_file() else None


def connect(path: Path) -> Any:
    try:
        from rvd_railnode.client import NodeClient
    except ImportError:
        raise RemoteError("the rail-node client is missing; reinstall rvd-railbench") from None
    return NodeClient.from_connection(path)


def remote_override(max_steps: int) -> str:
    """Last --config file of a run on the node."""
    rail = "".join(f"{key} = {value}\n" for key, value in REMOTE_TIMING.items())
    return f"[rail]\n{rail}[simulation]\nfast = false\nmax_steps = {max_steps}\n"


def move(node: Any, goal: dict[str, float], speed: dict[str, float] | float, cancel: threading.Event,
         progress: Callable[[dict[str, Any]], None] = lambda _: None) -> str:
    """One waypoint as a task under its own lease; the axes are released afterwards. Returns the job status."""
    node.acquire("task", list(goal), period_s=1.0)
    try:
        node.task([{"position": goal, "speed": speed}])
        while True:
            state = node.state()
            progress(state)
            job = state.get("job") or {}
            if job.get("status") != "running":
                return str(job.get("status", "lost"))
            if cancel.wait(0.2):
                node.stop()
                return "stopped"
    finally:
        node.release()


def held_axes(capabilities: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Axes the node keeps enabled when released, such as an attitude platform that would fall unpowered."""
    return {n: a for n, a in capabilities["axes"].items() if a.get("hold") and a.get("motion")}


def level(node: Any, cancel: threading.Event, progress: Callable[[dict[str, Any]], None] = lambda _: None) -> str:
    """Drives the held axes to zero at their park speed; the node keeps holding them there."""
    axes = held_axes(node.capabilities())
    if not axes:
        return "no held axes on this node"
    speed = {n: a.get("park_speed") or 0.2 * a["max_speed"] for n, a in axes.items()}
    return move(node, dict.fromkeys(axes, 0.0), speed, cancel, progress)


def park(node: Any, cancel: threading.Event, progress: Callable[[dict[str, Any]], None] = lambda _: None) -> str:
    """Drives the held axes to their park positions; the node releases them there."""
    reply = node.park()
    if not reply.get("axes"):
        return "nothing to park"
    while True:
        state = node.state()
        progress(state)
        job = state.get("job") or {}
        if job.get("id") != reply["task_id"] or job.get("status") != "running":
            break
        if cancel.wait(0.2):
            node.stop()
            return "stopped; the held axes keep holding"
    if job.get("id") == reply["task_id"] and job.get("status") == "completed":
        return "parked and released"
    return f"{job.get('status', 'lost')}: {job.get('reason') or state.get('fault') or state.get('reason')}"


class Bridge:
    """Holds a stream lease on x, y, z and answers rail_main's READ / SEND / HOLD lines (see bridge.hpp)."""

    def __init__(self, node: Any, log: Callable[[str], None] = lambda _: None) -> None:
        self.node = node
        self.log = log
        self.stop_event = threading.Event()
        self.finished = threading.Event()
        self.listener = socket.create_server(("127.0.0.1", 0))
        self.port = self.listener.getsockname()[1]
        self.axes: dict[str, Any] = {}
        self._released = threading.Lock()
        self._thread = threading.Thread(target=self._serve, name="rvdlab-bridge", daemon=True)

    def acquire(self, period_s: float = 1.0, timeout_s: float = 1.5) -> None:
        caps = self.node.capabilities()
        if not caps["motion_enabled"]:
            raise RemoteError("the node is read-only; restart it with --allow-motion")
        frozen = [a for a in AXES if a not in caps["axes"] or not caps["axes"][a]["motion"]]
        if frozen:
            raise RemoteError(f"the node does not move {', '.join(frozen)}")
        self.axes = caps["axes"]
        self.node.acquire("stream", AXES, period_s=period_s, timeout_s=timeout_s)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        """Stops the rail at once; rail_main learns of it on its next request."""
        self.stop_event.set()
        try:
            self.node.stop()
        except Exception as exc:  # the node may be unreachable; rail_main still stops on its next exchange
            self.log(f"stop request failed: {exc}")

    def close(self) -> None:
        self.stop_event.set()
        try:
            self.listener.close()
        except OSError:
            pass
        if self._thread.is_alive():
            self._thread.join(5)
        self._release()

    def _release(self) -> None:
        with self._released:
            if self.node.lease is not None:
                try:
                    self.node.release()
                except Exception as exc:
                    self.log(f"release failed, the node lease will expire: {exc}")

    def _serve(self) -> None:
        try:
            self.listener.settimeout(60)
            connection, _ = self.listener.accept()
        except OSError:
            self._release()
            self.finished.set()
            return
        try:
            with connection, connection.makefile("rw", encoding="ascii", newline="\n") as stream:
                for line in stream:
                    stream.write(self.answer(line.split()) + "\n")
                    stream.flush()
        except OSError:
            pass
        finally:
            self._release()
            self.finished.set()

    def answer(self, words: list[str]) -> str:
        try:
            if words == ["READ"]:
                return self._read()
            if len(words) == 8 and words[0] == "SEND":
                return self._send([float(w) for w in words[2:]])
            if words == ["HOLD"]:
                self._release()
                return "OK"
            return "ERR unknown request"
        except Exception as exc:
            return f"ERR {exc}".replace("\n", " ")

    def _read(self) -> str:
        begin = time.monotonic()
        self.node.clock()
        rtt = time.monotonic() - begin
        state = self.node.state(fresh=True)
        encoders = state["sensors"]["encoders"]
        if not encoders["ok"] or encoders["sample_time"] is None:
            return f"ERR encoders: {encoders['error']}"
        stop = self.stop_event.is_set() or state["control"] is None or bool(state["fault"])
        if stop and not self.stop_event.is_set():
            self.log(f"the node ended control: {state['fault'] or state['reason']}")
        age = state["server_time"] - encoders["sample_time"] + rtt / 2
        position = " ".join(f"{state['position'][a]:.9f}" for a in AXES)
        # The node clock may tick in 15.6 ms steps (Windows, Python < 3.13).
        return f"POS {position} {position} {age:.6f} {rtt / 2 + 0.016:.6f} {encoders['seq']} {int(stop)}"

    def _send(self, values: list[float]) -> str:
        if self.stop_event.is_set():
            return "ACK 1"
        target, velocity = values[:3], values[3:]
        speed = {a: max(abs(v), self.axes[a]["min_speed"]) for a, v in zip(AXES, velocity)}
        self.node.setpoint(dict(zip(AXES, target)), speed)
        return f"ACK {int(self.stop_event.is_set())}"
