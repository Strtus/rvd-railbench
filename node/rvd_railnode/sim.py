"""Simulated rail with PLC positioning semantics: each axis runs toward its target at the commanded speed,
stops there, and accepts a new target while moving."""

from __future__ import annotations

import math
import time
from typing import Any, Callable, Iterable

from .backend import BackendError, Sample
from .profile import axes as profile_axes


class SimBackend:
    name = "simulator"

    def __init__(self, profile: dict[str, Any], allow_motion: bool = True,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.profile = profile
        self.axes = profile_axes(profile)
        self.allow_motion = allow_motion
        self._clock = clock
        spec = profile["axes"]
        self._accel = {n: float(spec[n].get("acceleration", a.max_speed)) for n, a in self.axes.items()}
        self.position = {n: float(spec[n].get("start", min(max(0.0, a.minimum), a.maximum)))
                         for n, a in self.axes.items()}
        self.velocity = dict.fromkeys(self.axes, 0.0)
        self.target = dict(self.position)
        self.speed = dict.fromkeys(self.axes, 0.0)
        self.armed: set[str] = set()
        self.active = False
        self.connected = True
        self.status: dict[str, Any] = {}
        self.commands = 0
        # Fault injection for tests and demonstrations.
        self.enable_delay_s = 0.0
        self.fail_reads = 0
        self.fail_stop = False
        self.stuck: set[str] = set()
        self.dropped: set[str] = set()  # armed axes whose drive reports it is no longer enabled
        self.left_enabled: set[str] = set()  # axes found enabled at start, as if held through a restart
        self._t = clock()

    def _advance(self) -> None:
        now = self._clock()
        dt, self._t = max(0.0, now - self._t), now
        if dt == 0.0:
            return
        steps = min(4000, max(1, math.ceil(dt / 0.005)))
        h = dt / steps
        for name in self.axes:
            if name in self.stuck:
                self.velocity[name] = 0.0
                continue
            a, vmax = self._accel[name], self.speed[name]
            x, v, goal = self.position[name], self.velocity[name], self.target[name]
            for _ in range(steps):
                d = goal - x
                desired = math.copysign(min(vmax, math.sqrt(2.0 * a * abs(d))), d)
                v += max(-a * h, min(a * h, desired - v))
                if abs(v * h) >= abs(d) and v * d >= 0.0:
                    x, v = goal, 0.0
                    break
                x += v * h
            self.position[name], self.velocity[name] = x, v

    def read(self) -> Sample:
        begin = self._clock()
        if self.fail_reads:
            self.fail_reads -= 1
            raise BackendError("simulated feedback failure")
        self._advance()
        return Sample(dict(self.position), begin, self._clock() - begin, dropped=frozenset(self.dropped & self.armed))

    def _require_motion(self) -> None:
        if not self.allow_motion:
            raise BackendError("node is read-only")

    def arm(self, names: Iterable[str]) -> None:
        self._require_motion()
        self._advance()
        if self.enable_delay_s:
            time.sleep(self.enable_delay_s)
        self.active = True
        self.armed.update(names)

    def adopt(self, names: Iterable[str]) -> list[str]:
        if not self.allow_motion:
            return []
        found = sorted(set(names) & self.left_enabled - self.armed)
        self.armed.update(found)
        self.active = self.active or bool(found)
        return found

    def command(self, targets: dict[str, float], speeds: dict[str, float]) -> None:
        self._require_motion()
        self._advance()
        missing = [n for n in targets if n not in self.armed]
        if missing:
            raise BackendError(f"{', '.join(missing)} not armed")
        self.active = True
        for name, value in targets.items():
            self.target[name], self.speed[name] = float(value), float(speeds[name])
        self.commands += 1

    def stop(self, force: bool = False, keep: frozenset[str] | set[str] = frozenset()) -> None:
        if not self.allow_motion or not (self.active or force):
            return
        if self.fail_stop:
            raise BackendError("simulated stop failure")
        self._advance()
        holding = self.armed & set(keep)
        for name in self.axes:
            self.target[name], self.velocity[name] = self.position[name], 0.0
            if name not in holding:
                self.speed[name] = 0.0
        self.armed = holding
        self.active = bool(holding)

    def close(self) -> None:
        pass
