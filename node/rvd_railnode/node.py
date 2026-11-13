"""Node core: one control thread owns the hardware; request threads only exchange requests with it."""

from __future__ import annotations

import math
import queue
import secrets
import threading
import time
import uuid
from dataclasses import dataclass, fields
from typing import Any, Callable

from . import __version__
from .backend import InterlockError
from .profile import Axis

PROTOCOL = "rail-node/1"
MAX_WAYPOINTS = 1000


class Rejected(Exception):
    def __init__(self, message: str, status: int = 400) -> None:
        super().__init__(message)
        self.status = status


@dataclass
class Settings:
    poll_s: float = 0.2
    lease_ttl_s: float = 3.0
    min_period_s: float = 0.01
    max_period_s: float = 2.0
    max_timeout_s: float = 3.0
    horizon_s: float = 3.0  # the hardware is never given a target further than this many seconds of travel
    task_lead_s: float = 2.0
    task_refresh_s: float = 1.0
    task_timeout_s: float = 900.0
    stall_timeout_s: float = 3.0
    max_sample_age_s: float = 1.0
    drive_dropout_s: float = 1.0  # a drive's enabled report must be missing this long; it flickers while moving
    max_observation_age_s: float = 1.0
    arrival_samples: int = 3
    stop_retry_s: float = 1.0
    wait_s: float = 15.0

    @classmethod
    def from_profile(cls, profile: dict[str, Any], overrides: dict[str, Any] | None = None) -> Settings:
        values = {k: profile[k] for k in ("poll_s", "stall_timeout_s") if k in profile}
        values.update(overrides or {})
        known = {f.name for f in fields(cls)}
        unknown = set(values) - known
        if unknown:
            raise ValueError(f"unknown node settings: {', '.join(sorted(unknown))}")
        settings = cls(**values)
        settings.check()
        return settings

    def check(self) -> None:
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{f.name} must be a positive number")
        if not self.task_refresh_s < self.task_lead_s <= self.horizon_s:
            raise ValueError("require task_refresh_s < task_lead_s <= horizon_s")
        if not self.min_period_s < self.max_period_s < self.max_timeout_s:
            raise ValueError("require min_period_s < max_period_s < max_timeout_s")


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise Rejected(f"{name} must be a finite number")
    return float(value)


def _finite_or_none(value: float) -> float | None:
    return value if math.isfinite(value) else None


class _Request:
    def __init__(self, payload: Any = None) -> None:
        self.payload = payload
        self.done = threading.Event()
        self.error: Rejected | None = None
        self.result: Any = None

    def finish(self, result: Any = None, error: Rejected | None = None) -> None:
        if not self.done.is_set():
            self.result, self.error = result, error
            self.done.set()


class Node:
    def __init__(self, backend: Any, settings: Settings | None = None,
                 clock: Callable[[], float] = time.monotonic,
                 on_event: Callable[[str, str], None] | None = None, mocap: Any = None,
                 remote_reset: bool = False) -> None:
        self.backend = backend
        self.mocap = mocap
        self.remote_reset = remote_reset
        self.settings = settings or Settings()
        self.clock = clock
        self.on_event = on_event
        self.axes: dict[str, Axis] = backend.axes
        self.held = frozenset(n for n, a in self.axes.items() if a.hold)
        self.instance_id = uuid.uuid4().hex
        self.lock = threading.Lock()
        self._wake = threading.Condition(self.lock)
        self.position = dict.fromkeys(self.axes, math.nan)
        self.sample_time = math.nan
        self.sample_duration = math.nan
        self.feedback_error: str | None = "no sample yet"
        self.sample_seq = 0
        self._polls_started = 0
        self._polls_done = 0
        self.target: dict[str, float] = {}
        self.speed: dict[str, float] = {}
        self.armed: set[str] = set()
        self.device: dict[str, Any] = {}
        self.lease: dict[str, Any] | None = None
        self.stream_deadline: float | None = None
        self.last_seq = -1
        self.job: dict[str, Any] | None = None
        self.fault: str | None = None
        self.reason = "startup"
        self.revision = 0
        self._progress: dict[str, tuple[float, float]] = {}
        self._next_poll = -math.inf
        self._next_stop_retry = -math.inf
        self._stop_req: _Request | None = None
        self._stop_force = False
        self._stop_release: set[str] = set()  # held axes this stop releases (parked, or their drive is gone)
        self._resumed = -math.inf  # end of the last blocking hardware call; sample age counts from here at most
        self._dropped: frozenset[str] = frozenset()
        self._drop_since: dict[str, float] = {}
        self._dropped_reported: set[str] = set()
        self._abort_park = threading.Event()
        self._arm_req: _Request | None = None
        self._setpoint_req: _Request | None = None
        self._closing = False
        self._thread = threading.Thread(target=self._run, name="rail-node-control", daemon=True)
        # Event output (console, audit) runs apart, so a stalled console never holds up control.
        self._events: queue.SimpleQueue[tuple[str, str] | None] = queue.SimpleQueue()
        self._event_thread = threading.Thread(target=self._dispatch_events, name="rail-node-events", daemon=True)

    # lifecycle

    def start(self) -> None:
        if self.mocap:
            self.mocap.start()
        self._event_thread.start()
        self._thread.start()

    def close(self, timeout: float = 120.0) -> None:
        with self.lock:
            self._closing = True
            self._wake.notify_all()
        end = time.monotonic() + timeout
        try:
            while self._thread.is_alive() and time.monotonic() < end:
                self._thread.join(0.2)
        except KeyboardInterrupt:  # a second Ctrl+C cuts the shutdown park short; held axes stay enabled
            self._abort_park.set()
            self._thread.join(15.0)
        if self.mocap:
            self.mocap.close()
        self._events.put(None)
        if self._event_thread.is_alive():
            self._event_thread.join(2.0)

    def _event(self, kind: str, detail: str) -> None:
        if self.on_event:
            self._events.put((kind, detail))

    def _dispatch_events(self) -> None:
        while True:
            item = self._events.get()
            if item is None:
                return
            try:
                if self.on_event:
                    self.on_event(*item)
            except Exception:
                pass

    # views (no hardware access)

    def capabilities(self) -> dict[str, Any]:
        s = self.settings
        return {
            "protocol": PROTOCOL,
            "version": __version__,
            "instance_id": self.instance_id,
            "backend": self.backend.name,
            "hardware_connected": bool(self.backend.connected),
            "motion_enabled": bool(self.backend.allow_motion),
            "axes": {n: a.describe() for n, a in self.axes.items()},
            "lease_ttl_s": s.lease_ttl_s,
            "period_s": [s.min_period_s, s.max_period_s],
            "max_timeout_s": s.max_timeout_s,
            "horizon_s": s.horizon_s,
            "task": {"lead_s": s.task_lead_s, "refresh_s": s.task_refresh_s, "max_points": MAX_WAYPOINTS},
            "sensors": ["encoders"] + (["mocap"] if self.mocap else []),
            "mocap": None if not self.mocap else {
                "source": f"vrpn://{self.mocap.host}:{self.mocap.port}", "trackers": list(self.mocap.trackers),
                "rail_frame": [n for n, t in self.mocap.trackers.items() if "to_rail" in t]},
        }

    def snapshot(self, fresh: bool = False) -> dict[str, Any]:
        with self.lock:
            if fresh:  # wait for a poll that starts after this request; counted, as clocks may be coarse
                wanted = self._polls_started + 1
                self._next_poll = -math.inf
                self._wake.notify_all()
                end = self.clock() + self.settings.wait_s
                while not self._closing and self._polls_done < wanted and self.clock() < end:
                    self._wake.wait(end - self.clock())
            return self._snapshot()

    def _snapshot(self) -> dict[str, Any]:
        now = self.clock()
        moving = any(abs(t - self.position[n]) > self.axes[n].tolerance for n, t in self.target.items()
                     if math.isfinite(self.position[n]))
        state = ("fault" if self.fault else "parking" if self.lease and self.lease["mode"] == "park" else
                 "moving" if moving else ("armed" if self.lease else "holding") if self.backend.active else "idle")
        lease, job = self.lease, self.job
        age = now - self.sample_time if math.isfinite(self.sample_time) else math.nan
        return {
            "protocol": PROTOCOL,
            "instance_id": self.instance_id,
            "server_time": now,
            "revision": self.revision,
            "backend": self.backend.name,
            "hardware_connected": bool(self.backend.connected),
            "motion_enabled": bool(self.backend.allow_motion),
            "state": state,
            "fault": self.fault,
            "reason": self.reason,
            "position": {n: _finite_or_none(v) for n, v in self.position.items()},
            "target": dict(self.target),
            "speed": dict(self.speed),
            "armed": sorted(self.armed),
            "sensors": {"encoders": {
                "ok": self.feedback_error is None,
                "error": self.feedback_error,
                "sample_time": _finite_or_none(self.sample_time),
                "age_s": _finite_or_none(age),
                "read_s": _finite_or_none(self.sample_duration),
                "seq": self.sample_seq,
                "values": {n: _finite_or_none(v) for n, v in self.position.items()},
            }, **({"mocap": self.mocap.state()} if self.mocap else {})},
            "control": None if lease is None else {
                "owner": lease["owner"], "mode": lease["mode"], "axes": list(lease["axes"]),
                "armed": lease["armed"], "period_s": lease["period_s"], "timeout_s": lease["timeout_s"],
                "remaining_s": max(0.0, lease["expires"] - now) if math.isfinite(lease["expires"]) else None,
            },
            "job": None if job is None else {
                "id": job["id"], "status": job["status"], "waypoint": job["waypoint"], "count": job["count"],
                "reason": job["reason"], "elapsed_s": now - job["started"],
            },
            "device": dict(self.device),
        }

    # requests from clients

    def _owned(self, user: str, data: dict[str, Any], mode: str | None = None) -> dict[str, Any]:
        self._expire()
        lease = self.lease
        if lease is None or lease["owner"] != user or not secrets.compare_digest(
                str(data.get("lease_id", "")).encode(), lease["id"].encode()):
            raise Rejected("a valid control lease is required", 409)
        if not lease["armed"]:
            raise Rejected("control is still being armed", 409)
        if mode and lease["mode"] != mode:
            raise Rejected(f"this request needs a {mode} lease", 409)
        lease["expires"] = self.clock() + self.settings.lease_ttl_s  # every valid request also keeps control alive
        return lease

    def _check_point(self, data: dict[str, Any], allowed: list[str], horizon: bool) -> tuple[dict, dict]:
        position, speed = data.get("position"), data.get("speed")
        if not isinstance(position, dict) or not position:
            raise Rejected("position must map axis names to targets")
        outside = [n for n in position if n not in allowed]
        if outside:
            raise Rejected(f"{', '.join(map(str, outside))} not part of this lease")
        if isinstance(speed, dict) and set(speed) != set(position):
            raise Rejected("speed must give one value per axis in position")
        targets, speeds = {}, {}
        for name, value in position.items():
            axis = self.axes[name]
            target = _number(value, f"position.{name}")
            v = _number(speed[name] if isinstance(speed, dict) else speed, f"speed.{name}")
            if not axis.minimum <= target <= axis.maximum:
                raise Rejected(f"{name} target {target:g} is outside [{axis.minimum:g}, {axis.maximum:g}]")
            if not axis.min_speed <= v <= axis.max_speed:
                raise Rejected(f"{name} speed {v:g} is outside [{axis.min_speed:g}, {axis.max_speed:g}]")
            if horizon:
                here = self.position[name]
                if not math.isfinite(here) or self.feedback_error:
                    raise Rejected("position feedback is unavailable", 503)
                reach = v * self.settings.horizon_s + axis.tolerance
                if abs(target - here) > reach:
                    raise Rejected(f"{name} target is {abs(target - here):.4g} away; at {v:g}/s that is beyond "
                                   f"the {self.settings.horizon_s:g} s horizon")
            targets[name], speeds[name] = target, v
        return targets, speeds

    def acquire(self, user: str, data: dict[str, Any]) -> dict[str, Any]:
        s = self.settings
        mode = data.get("mode")
        if mode not in ("task", "stream"):
            raise Rejected("mode must be task or stream")
        names = data.get("axes")
        if (not isinstance(names, list) or not names or any(not isinstance(n, str) or n not in self.axes for n in names)
                or len(set(names)) != len(names)):
            raise Rejected(f"axes must list distinct axes of this node: {', '.join(self.axes)}")
        frozen = [n for n in names if not self.axes[n].motion]
        if frozen:
            raise Rejected(f"{', '.join(frozen)} does not accept motion on this node", 409)
        period = _number(data.get("period_s", 1.0), "period_s")
        timeout = _number(data.get("timeout_s", min(s.max_timeout_s, 1.5 * period)), "timeout_s")
        if not s.min_period_s <= period <= s.max_period_s or not period < timeout <= s.max_timeout_s:
            raise Rejected(f"require {s.min_period_s:g} <= period_s <= {s.max_period_s:g} "
                           f"and period_s < timeout_s <= {s.max_timeout_s:g}")
        with self.lock:
            self._expire()
            if not self.backend.allow_motion:
                raise Rejected("node is read-only", 409)
            if self.fault:
                raise Rejected(f"node is faulted: {self.fault}", 409)
            if self.lease:
                raise Rejected(f"control is held by {self.lease['owner']}", 409)
            if self._stop_req:
                raise Rejected("a stop is in progress", 409)
            if self.feedback_error:
                raise Rejected(f"hardware feedback is unavailable: {self.feedback_error}", 503)
            lease = {"id": secrets.token_urlsafe(24), "owner": user, "mode": mode, "axes": list(names),
                     "period_s": period, "timeout_s": timeout, "expires": math.inf, "armed": False,
                     "created": self.clock()}
            self.lease, self.last_seq, self.stream_deadline = lease, -1, None
            if self.job and self.job["status"] != "running":
                self.job = None
            request = self._arm_req = _Request(list(names))
            self.revision += 1
            self._wake.notify_all()
        self._event("lease", f"{user} requested {mode} control of {' '.join(names)}")
        try:
            self._await(request)
        except Rejected:
            with self.lock:
                if self.lease is lease:
                    self._request_stop("arm_failed", force=True, revoke=True)
            raise
        with self.lock:
            if self.lease is not lease:
                raise Rejected("control was revoked while arming", 409)
            now = self.clock()
            lease["armed"], lease["expires"] = True, now + s.lease_ttl_s
            self.revision += 1
            return {"lease_id": lease["id"], "mode": mode, "axes": list(names), "period_s": period,
                    "timeout_s": timeout, "ttl_s": s.lease_ttl_s, "instance_id": self.instance_id,
                    "server_time": now}

    def heartbeat(self, user: str, data: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            lease = self._owned(user, data)
            now = self.clock()
            lease["expires"] = now + self.settings.lease_ttl_s
            return {"ttl_s": self.settings.lease_ttl_s, "server_time": now}

    def release(self, user: str, data: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            self._owned(user, data)
            request = self._request_stop("released", force=False, revoke=False)
        try:
            self._await(request)
        finally:
            with self.lock:
                if self.lease and self.lease["owner"] == user:
                    self.lease = None
                    self.revision += 1
        self._event("lease", f"{user} released control")
        return {"released": True}

    def setpoint(self, user: str, data: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            lease = self._owned(user, data, "stream")
            if self.fault:
                raise Rejected(f"node is faulted: {self.fault}", 409)
            now = self.clock()
            seq = data.get("seq")
            if type(seq) is not int or seq <= self.last_seq:
                raise Rejected("seq must be an increasing integer", 409)
            if "instance_id" in data and data["instance_id"] != self.instance_id:
                raise Rejected("the node restarted", 409)
            deadline = _number(data.get("deadline"), "deadline")
            if not now < deadline <= now + lease["timeout_s"]:
                raise Rejected("deadline has passed or exceeds the negotiated timeout", 409)
            if "sample_time" in data:
                age = now - _number(data["sample_time"], "sample_time")
                if not 0 <= age <= self.settings.max_observation_age_s:
                    raise Rejected(f"the observation is {age:.3f} s old", 409)
            targets, speeds = self._check_point(data, lease["axes"], horizon=True)
            self.last_seq = seq
            if self._setpoint_req:
                self._setpoint_req.finish(error=Rejected("superseded by a newer setpoint", 409))
            request = self._setpoint_req = _Request((seq, targets, speeds, deadline, now))
            self._wake.notify_all()
        return self._await(request, max(0.0, deadline - self.clock()) + 0.5)

    def submit_task(self, user: str, data: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            lease = self._owned(user, data, "task")
            if self.fault:
                raise Rejected(f"node is faulted: {self.fault}", 409)
            if self.job and self.job["status"] == "running":
                raise Rejected("a task is already running", 409)
            points = data.get("points")
            if not isinstance(points, list) or not 1 <= len(points) <= MAX_WAYPOINTS:
                raise Rejected(f"points must hold 1..{MAX_WAYPOINTS} waypoints")
            if any(not isinstance(p, dict) for p in points):
                raise Rejected("each waypoint must be an object")
            checked = [self._check_point(p, lease["axes"], horizon=False) for p in points]
            now = self.clock()
            self.job = {"id": uuid.uuid4().hex, "status": "running", "waypoint": 0, "count": len(checked),
                        "points": checked, "started": now, "reason": None, "refreshed": -math.inf,
                        "dirty": True, "arrivals": 0, "checked_sample": self.sample_seq}
            self.reason = "task"
            self.revision += 1
            self._wake.notify_all()
            job_id = self.job["id"]
        self._event("task", f"{user} started a task with {len(checked)} waypoints")
        return {"task_id": job_id, "status": "running"}

    def stop(self, user: str) -> dict[str, Any]:
        with self.lock:
            request = self._request_stop(f"operator_stop:{user}", force=True, revoke=True)
        self._event("stop", f"stop requested by {user}")
        self._await(request)
        return self.snapshot()

    def reset(self, user: str, local: bool) -> dict[str, Any]:
        if not local and not self.remote_reset:
            raise Rejected("a fault can only be reset on the node computer", 403)
        with self.lock:
            if not self.fault:
                return self._snapshot()
            # A held axis whose drive dropped out holds nothing any more, so the reset lets it go.
            request = self._request_stop(f"reset:{user}", force=True, revoke=True, release=self._dropped & self.held)
        self._await(request)
        with self.lock:
            if self._unsettled():
                raise Rejected("axes are still active", 409)
            cleared, self.fault = self.fault, None
            self._dropped_reported.clear()
            self.reason = f"reset:{user}"
            self.revision += 1
            snapshot = self._snapshot()
        self._event("reset", f"{user} cleared: {cleared}")
        return snapshot

    def park(self, user: str) -> dict[str, Any]:
        """Drives the held axes to their park positions, then releases them; they rest there unpowered."""
        s = self.settings
        with self.lock:
            self._expire()
            if not self.backend.allow_motion:
                raise Rejected("node is read-only", 409)
            if self.fault:
                raise Rejected(f"node is faulted: {self.fault}", 409)
            if self.lease:
                raise Rejected(f"control is held by {self.lease['owner']}", 409)
            if self._stop_req:
                raise Rejected("a stop is in progress", 409)
            if self.feedback_error:
                raise Rejected(f"hardware feedback is unavailable: {self.feedback_error}", 503)
            names = self._parkable()
            if not names:
                return {"status": "idle", "axes": []}
            now = self.clock()
            goal = {n: self.axes[n].park for n in names}
            speeds = {n: self.axes[n].park_speed for n in names}
            self.lease = {"id": secrets.token_urlsafe(24), "owner": user, "mode": "park", "axes": names,
                          "period_s": s.task_refresh_s, "timeout_s": s.max_timeout_s, "expires": math.inf,
                          "armed": True, "created": now}
            self.job = {"id": uuid.uuid4().hex, "status": "running", "waypoint": 0, "count": 1,
                        "points": [(goal, speeds)], "started": now, "reason": None, "refreshed": -math.inf,
                        "dirty": True, "arrivals": 0, "checked_sample": self.sample_seq, "park": names}
            self.reason = "park"
            self.revision += 1
            self._wake.notify_all()
            job_id = self.job["id"]
        self._event("park", f"{user} parks {' '.join(names)}")
        return {"task_id": job_id, "status": "running", "axes": names}

    def _sustained(self, since: dict[str, float], dropped: frozenset[str], now: float) -> frozenset[str]:
        """The drives whose enabled report has been missing for drive_dropout_s."""
        for name in list(since):
            if name not in dropped:
                del since[name]
        for name in dropped:
            since.setdefault(name, now)
        return frozenset(n for n, t in since.items() if now - t >= self.settings.drive_dropout_s)

    def _parkable(self) -> list[str]:
        armed = set(getattr(self.backend, "armed", ()))
        return sorted(n for n in self.held if n in armed and self.axes[n].park is not None)

    def _unsettled(self) -> bool:
        """True while any axis other than a held one is still active."""
        return bool(self.backend.active and set(getattr(self.backend, "armed", ())) - self.held)

    def _await(self, request: _Request, timeout: float | None = None) -> Any:
        if not request.done.wait(self.settings.wait_s if timeout is None else timeout):
            raise Rejected("the node did not complete the request in time", 503)
        if request.error:
            raise request.error
        return request.result

    # supervision (caller holds the lock)

    def _request_stop(self, reason: str, force: bool, revoke: bool, fault: str | None = None,
                      release: frozenset[str] | set[str] = frozenset()) -> _Request:
        if fault and not self.fault:
            self.fault = fault
            self._event("fault", fault)
        # Held axes stay enabled through every stop, faults included: unpowered they would fall. Only parking
        # (or a drive that is already gone) releases them.
        self._stop_release.update(release)
        self.reason = reason
        self.stream_deadline = None
        if self.job and self.job["status"] == "running":
            self.job["status"], self.job["reason"] = "stopped", reason
        for pending in (self._arm_req, self._setpoint_req):
            if pending:
                pending.finish(error=Rejected(f"stopped: {reason}", 409))
        self._arm_req = self._setpoint_req = None
        if revoke and self.lease:
            self.lease = None
        if self._stop_req is None:
            self._stop_req = _Request(reason)
        self._stop_force = self._stop_force or force
        self.revision += 1
        self._wake.notify_all()
        return self._stop_req

    def _at_target(self, name: str) -> bool:
        target = self.target.get(name)
        return target is None or abs(target - self.position[name]) <= self.axes[name].tolerance

    def _reverses(self, name: str, target: float) -> bool:
        old, here = self.target.get(name), self.position[name]
        return old is not None and (old - here) * (target - here) < 0

    def _progress_step(self, name: str, speed: float | None = None) -> float:
        axis = self.axes[name]
        expected = 0.5 * (self.speed.get(name, 0.0) if speed is None else speed) * self.settings.stall_timeout_s
        return max(2 * axis.resolution, min(axis.tolerance, expected) if expected > 0 else axis.tolerance)

    def _carrot(self, name: str, here: float, goal: float, speed: float) -> float:
        """A target at most task_lead_s of travel ahead, within the limits even if the axis rests outside them."""
        reach = speed * self.settings.task_lead_s
        carrot = goal if abs(goal - here) <= reach else here + math.copysign(reach, goal - here)
        return min(max(carrot, self.axes[name].minimum), self.axes[name].maximum)

    def _expire(self) -> None:
        now = self.clock()
        lease = self.lease
        if lease and not lease["armed"] and now - lease["created"] > self.settings.wait_s + 5.0:
            self._request_stop("arm_timeout", force=True, revoke=True)
            self._event("lease", f"control of {lease['owner']} was never armed; revoked")
        elif lease and lease["armed"] and now >= lease["expires"]:
            self._request_stop("lease_expired", force=False, revoke=True)
            self._event("lease", f"control of {lease['owner']} expired")
        elif self.stream_deadline is not None and now >= self.stream_deadline:
            self._request_stop("stream_timeout", force=False, revoke=True)
            self._event("lease", "setpoint stream timed out")

    def _monitor(self) -> None:
        # Control thread only, right after a poll, so slow hardware calls never age the sample it judges.
        now, s = self.clock(), self.settings
        if self.fault and self._unsettled() and not self._stop_req and now >= self._next_stop_retry:
            self._next_stop_retry = now + s.stop_retry_s
            self._request_stop("stop_retry", force=True, revoke=True)
        if not self.backend.active or self._stop_req or self.fault:
            return
        fresh = max(self.sample_time if math.isfinite(self.sample_time) else -math.inf, self._resumed)
        if now - fresh > s.max_sample_age_s:
            cause = self.feedback_error or "no new sample"
            self._request_stop("feedback_lost", force=True, revoke=True,
                               fault=f"FEEDBACK_LOST: {cause} (no position for {s.max_sample_age_s:g} s)")
            return
        for name in self.target:
            if self._at_target(name):
                self._progress[name] = (now, self.position[name])  # time spent waiting at the target is no stall
                continue
            mark = self._progress.get(name)
            if mark and now - mark[0] > s.stall_timeout_s:
                self._request_stop("stalled", force=True, revoke=True,
                                   fault=f"MOTION_STALLED: {name} made no progress for {s.stall_timeout_s:g} s")
                return
        job = self.job
        if job and job["status"] == "running":
            if now - job["started"] > s.task_timeout_s:
                self._request_stop("task_timeout", force=False, revoke=True)
            elif self.sample_seq != job["checked_sample"]:
                job["checked_sample"] = self.sample_seq
                goal, _ = job["points"][job["waypoint"]]
                there = all(abs(v - self.position[n]) <= self.axes[n].tolerance for n, v in goal.items())
                job["arrivals"] = job["arrivals"] + 1 if there and not job["dirty"] else 0
                if job["arrivals"] >= s.arrival_samples:
                    job["waypoint"] += 1
                    job["arrivals"] = 0
                    if job["waypoint"] == job["count"]:
                        job["status"] = "completed"
                        if job.get("park"):
                            self._event("park", f"{' '.join(job['park'])} parked; releasing them")
                            self._request_stop("parked", force=False, revoke=True, release=job["park"])
                        else:
                            self._event("task", "task completed")
                    else:
                        job["dirty"] = True
                    self.revision += 1

    # control thread

    def _pending(self) -> bool:
        return bool(self._stop_req or self._arm_req or self._setpoint_req)

    def _run(self) -> None:
        self._adopt()
        while True:
            with self.lock:
                while not self._closing and not self._pending() and self.clock() < self._next_poll:
                    self._wake.wait(max(0.0, self._next_poll - self.clock()))
                if self._closing:
                    break
            try:
                self._step()
            except Exception as exc:  # the control thread must outlive any single failure
                with self.lock:
                    self._request_stop("internal_error", force=True, revoke=True, fault=f"INTERNAL_ERROR: {exc!r}")
                time.sleep(0.05)
        self._shutdown()

    def _step(self) -> None:
        with self.lock:
            due = self.clock() >= self._next_poll
            if due:
                self._next_poll = self.clock() + self.settings.poll_s
                self._polls_started += 1
        if due:
            self._poll()
        with self.lock:
            self._expire()
            self._monitor()
            stopping = self._stop_req is not None
        if stopping:
            self._service_stop()
            return
        self._service_arm()
        self._service_setpoint()
        self._service_task()

    def _poll(self) -> None:
        try:
            sample, error = self.backend.read(), None
        except Exception as exc:
            sample, error = None, exc
        with self.lock:
            self._polls_done += 1
            self._wake.notify_all()
            self.device = dict(self.backend.status)
            if sample is None:
                message = str(error) or type(error).__name__
                if self.feedback_error != message:
                    self._event("feedback", message)
                self.feedback_error = message
                # An interlock stops at once; a lost sample is tolerated until max_sample_age_s.
                if isinstance(error, InterlockError) and self.backend.active and not self._stop_req:
                    self._request_stop("interlock", force=True, revoke=True, fault=f"INTERLOCK: {message}")
                self.revision += 1
                return
            if self.feedback_error and self.feedback_error != "no sample yet":
                self._event("feedback", "restored")
            self.feedback_error = None
            dropped = self._sustained(self._drop_since, frozenset(sample.dropped), sample.time)
            fresh_drop = dropped - self._dropped_reported
            self._dropped, self._dropped_reported = dropped, (self._dropped_reported & dropped) | fresh_drop
            if fresh_drop and self.backend.active:
                self._request_stop("drive_lost", force=True, revoke=True,
                                   fault=f"DRIVE_DISABLED: {' '.join(sorted(dropped))} no longer reports enabled")
            for name, value in sample.positions.items():
                mark = self._progress.get(name)
                if mark is None or abs(value - mark[1]) >= self._progress_step(name):
                    self._progress[name] = (sample.time, value)
                self.position[name] = value
            self.sample_time, self.sample_duration = sample.time, sample.duration
            self.sample_seq += 1
            self.revision += 1

    def _command(self, targets: dict[str, float], speeds: dict[str, float]) -> float:
        with self.lock:
            # The stall clock restarts for an axis that was at its target or is now sent back the way it came.
            fresh = [n for n in targets if self._at_target(n) or self._reverses(n, targets[n])]
        self.backend.command(targets, speeds)
        with self.lock:
            now = self.clock()
            for name in targets:
                self.target[name], self.speed[name] = targets[name], speeds[name]
            for name in fresh:
                self._progress[name] = (now, self.position[name])
            self.revision += 1
            return now

    def _service_stop(self) -> None:
        # Detach the request first: a stop asked for while this one runs gets its own pass.
        with self.lock:
            request, force, release = self._stop_req, self._stop_force, self._stop_release
            self._stop_req, self._stop_force, self._stop_release = None, False, set()
        if request is None:
            return
        try:
            self.backend.stop(force=force, keep=self.held - release)
            error = None
        except Exception as exc:
            error = exc
        with self.lock:
            self._resumed = self.clock()
            self.target.clear()
            self.speed.clear()
            self.armed = set(getattr(self.backend, "armed", ())) if self.backend.active else set()
            if error is not None:
                message = f"STOP_UNCONFIRMED: {error}"
                self.fault = message if not self.fault else f"{self.fault}; {message}"
                self._next_stop_retry = self.clock() + self.settings.stop_retry_s
                self._event("fault", message)
            self.revision += 1
            failure = Rejected(self.fault or "stop failed", 503) if error is not None else None
        request.finish(True, failure)

    def _service_arm(self) -> None:
        with self.lock:
            request, self._arm_req = self._arm_req, None
        if request is None:
            return
        try:
            self.backend.arm(request.payload)
        except Exception as exc:
            with self.lock:
                self._resumed = self.clock()
                self._request_stop("arm_failed", force=True, revoke=True)
            request.finish(error=Rejected(f"arming failed: {exc}", 503))
            return
        with self.lock:
            self._resumed = self.clock()  # arming blocks polling, enabling a drive can take seconds
            self.armed.update(request.payload)
            self.revision += 1
        request.finish(True)

    def _service_setpoint(self) -> None:
        with self.lock:
            request, self._setpoint_req = self._setpoint_req, None
            if request is None:
                return
            seq, targets, speeds, deadline, received = request.payload
            lease = self.lease
            if lease is None or self.fault:
                request.finish(error=Rejected("control was revoked", 409))
                return
            if self.clock() >= deadline:
                request.finish(error=Rejected("the setpoint expired before it could be applied", 409))
                return
        try:
            applied = self._command(targets, speeds)
        except Exception as exc:
            with self.lock:
                self._request_stop("command_failed", force=True, revoke=True, fault=f"COMMAND_FAILED: {exc}")
            request.finish(error=Rejected(f"command failed: {exc}", 503))
            return
        with self.lock:
            if self.lease is lease:
                self.stream_deadline = deadline
            self.reason = "stream"
        request.finish({"accepted_seq": seq, "t_received": received, "t_applied": applied, "server_time": applied})

    def _service_task(self) -> None:
        s = self.settings
        with self.lock:
            job = self.job
            if not job or job["status"] != "running" or self._stop_req or self.fault or self.lease is None:
                return
            goal, speeds = job["points"][job["waypoint"]]
            if self.feedback_error or not all(math.isfinite(self.position[n]) for n in goal):
                return
            now = self.clock()
            if not job["dirty"] and now - job["refreshed"] < s.task_refresh_s:
                return
            targets = {n: self._carrot(n, self.position[n], v, speeds[n]) for n, v in goal.items()}
            unchanged = all(abs(targets[n] - self.target.get(n, math.inf)) <= 0.5 * self.axes[n].tolerance
                            and speeds[n] == self.speed.get(n) for n in targets)
            job["refreshed"] = now
            if unchanged and not job["dirty"]:
                return
            job["dirty"] = False
        try:
            self._command(targets, speeds)
        except Exception as exc:
            with self.lock:
                self._request_stop("command_failed", force=True, revoke=True, fault=f"COMMAND_FAILED: {exc}")

    def _adopt(self) -> None:
        """Held axes still enabled from an earlier run (a park that was cut short) are held again, not dropped."""
        if not self.held or not self.backend.allow_motion or not hasattr(self.backend, "adopt"):
            return
        try:
            found = self.backend.adopt(sorted(self.held))
        except Exception as exc:
            self._event("hold", f"cannot check whether {' '.join(sorted(self.held))} are enabled: {exc}")
            return
        if found:
            with self.lock:
                self.armed.update(found)
                self.reason = "adopted"
                self.revision += 1
            self._event("hold", f"{' '.join(found)} were still enabled; holding them")

    def _shutdown(self) -> None:
        with self.lock:
            for pending in (self._arm_req, self._setpoint_req, self._stop_req):
                if pending:
                    pending.finish(error=Rejected("node is shutting down", 503))
            self.lease = None
            if self.job and self.job["status"] == "running":
                self.job["status"], self.job["reason"] = "stopped", "shutdown"
        parked = self._park_for_shutdown()
        keep = self.held - parked
        try:
            self.backend.stop(force=False, keep=keep)
        except Exception as exc:
            self._event("fault", f"STOP_UNCONFIRMED at shutdown: {exc}")
        left = sorted(set(getattr(self.backend, "armed", ())) & keep)
        if left:
            self._event("hold", f"{' '.join(left)} stay enabled where they are")
        try:
            self.backend.close()
        except Exception:
            pass

    def _park_for_shutdown(self) -> frozenset[str]:
        """Drives the held axes to their park positions on the control thread; on any trouble they keep holding."""
        names = self._parkable() if self.backend.allow_motion else []
        if not names:
            return frozenset()
        s = self.settings
        goal = {n: self.axes[n].park for n in names}
        speeds = {n: self.axes[n].park_speed for n in names}
        self._event("park", f"parking {' '.join(names)} before shutdown (second Ctrl+C: stop and keep holding)")
        reason = None
        try:
            sample = self.backend.read()
        except Exception as exc:
            sample, reason = None, f"no feedback: {exc}"
        if sample is not None:
            now = self.clock()
            travel = max(abs(goal[n] - sample.positions[n]) / speeds[n] for n in names)
            deadline = now + travel + 10.0 * s.stall_timeout_s
            marks = {n: (now, sample.positions[n]) for n in names}
            arrivals, last_command, since = 0, -math.inf, {}
            while reason is None:
                here, now = sample.positions, self.clock()
                lost = sorted(self._sustained(since, frozenset(sample.dropped) & set(names), now))
                if self._abort_park.is_set():
                    reason = "interrupted"
                elif lost:
                    reason = f"{' '.join(lost)} no longer enabled"
                elif now > deadline:
                    reason = "timed out"
                if reason:
                    break
                arrived = all(abs(goal[n] - here[n]) <= self.axes[n].tolerance for n in names)
                arrivals = arrivals + 1 if arrived else 0
                if arrivals >= s.arrival_samples:
                    self._event("park", f"{' '.join(names)} parked")
                    return frozenset(names)
                for n in names:
                    if abs(here[n] - marks[n][1]) >= self._progress_step(n, speeds[n]):
                        marks[n] = (now, here[n])
                    elif abs(goal[n] - here[n]) > self.axes[n].tolerance and now - marks[n][0] > s.stall_timeout_s:
                        reason = f"{n} made no progress"
                if reason:
                    break
                if now - last_command >= s.task_refresh_s:
                    targets = {n: self._carrot(n, here[n], goal[n], speeds[n]) for n in names}
                    try:
                        self.backend.command(targets, speeds)
                    except Exception as exc:
                        reason = f"command failed: {exc}"
                        break
                    last_command = now
                time.sleep(s.poll_s)
                try:
                    sample = self.backend.read()
                except Exception as exc:
                    reason = f"no feedback: {exc}"
        self._event("park", f"parking stopped ({reason}); {' '.join(names)} keep holding")
        try:
            self.backend.stop(force=True, keep=self.held)
        except Exception as exc:
            self._event("fault", f"STOP_UNCONFIRMED while parking: {exc}")
        return frozenset()
