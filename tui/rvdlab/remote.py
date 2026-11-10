"""Runs on a rail node: the user's connection, manual moves, and the bridge that relays rail_main to the node."""

from __future__ import annotations

import math
import os
import socket
import threading
import time
from pathlib import Path
from typing import Any, Callable

AXES = ("x", "y", "z")
# A run through the network and a PLC needs looser timing than a simulated one; rail_main still checks them.
REMOTE_TIMING = {"clock_skew_s": 0.1, "communication_timeout_s": 0.8, "measurement_late_s": 0.5}
# A pulse axis started at a crawl keeps the crawl's acceleration for the rest of the move, so commands to the
# rail never carry less than this fraction of the axis speed limit.
SPEED_FLOOR = 0.1


class SimAxis:
    """An axis the bridge integrates itself: it runs at the commanded speed and stops at the target."""

    def __init__(self, position: float, now: float) -> None:
        self.position, self.target, self.speed, self.since = position, position, 0.0, now

    def at(self, now: float) -> float:
        step = self.speed * max(0.0, now - self.since)
        gap = self.target - self.position
        return self.target if abs(gap) <= step else self.position + math.copysign(step, gap)

    def command(self, target: float, speed: float, now: float) -> None:
        self.position, self.target, self.speed, self.since = self.at(now), target, speed, now


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
    """Holds a stream lease on x, y, z and answers rail_main's READ / SEND / HOLD lines (see bridge.hpp).

    Axes named in simulated stay where they are on the rail; the bridge integrates them itself instead."""

    def __init__(self, node: Any, log: Callable[[str], None] = lambda _: None, simulated: tuple[str, ...] = ()) -> None:
        self.node = node
        self.log = log
        self.simulated = tuple(a for a in AXES if a in simulated)
        self.real = tuple(a for a in AXES if a not in simulated)
        self.sim: dict[str, SimAxis] = {}
        self.floor: dict[str, float] = {}
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
        frozen = [a for a in self.real if a not in caps["axes"] or not caps["axes"][a]["motion"]]
        if frozen:
            raise RemoteError(f"the node does not move {', '.join(frozen)}")
        self.axes = caps["axes"]
        self.floor = {a: max(SPEED_FLOOR * self.axes[a]["max_speed"], self.axes[a]["min_speed"]) for a in self.real}
        if self.simulated:
            here, now = self.node.state(fresh=True)["position"], time.monotonic()
            self.sim = {a: SimAxis(float(here[a]), now) for a in self.simulated}
        self.node.acquire("stream", self.real, period_s=period_s, timeout_s=timeout_s)

    def describe(self) -> dict[str, Any]:
        return {"simulated_axes": list(self.simulated),
                "simulated_start": {a: s.position for a, s in self.sim.items()} if self.sim else {},
                "simulated_model": "commanded speed, stops at the target" if self.sim else None,
                "speed_floor_m_s": self.floor}

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
        now = time.monotonic()
        here = {a: self.sim[a].at(now) if a in self.sim else state["position"][a] for a in AXES}
        position = " ".join(f"{here[a]:.9f}" for a in AXES)
        # The node clock may tick in 15.6 ms steps (Windows, Python < 3.13).
        return f"POS {position} {position} {age:.6f} {rtt / 2 + 0.016:.6f} {encoders['seq']} {int(stop)}"

    def _send(self, values: list[float]) -> str:
        if self.stop_event.is_set():
            return "ACK 1"
        target, velocity = dict(zip(AXES, values[:3])), dict(zip(AXES, values[3:]))
        now = time.monotonic()
        for a, axis in self.sim.items():
            axis.command(target[a], abs(velocity[a]), now)
        speed = {a: max(abs(velocity[a]), self.floor[a]) for a in self.real}
        self.node.setpoint({a: target[a] for a in self.real}, speed)
        return f"ACK {int(self.stop_event.is_set())}"
