"""Trajectory CSV (the rail_main --log format, also written by the 1107 lab build) and run statistics."""

from __future__ import annotations

import csv
import math
import statistics
from dataclasses import dataclass, field
from pathlib import Path

from .config import RailConfig, Vec3


def num(row: dict[str, str], name: str) -> float | None:
    value = row.get(name)
    if not value:
        return None
    try:
        x = float(value)
    except ValueError:
        return None
    return x if math.isfinite(x) else None


def norm3(row: dict[str, str], a: str, b: str, c: str) -> float | None:
    values = [num(row, a), num(row, b), num(row, c)]
    if any(v is None for v in values):
        return None
    return math.sqrt(sum(v * v for v in values))  # type: ignore[operator]


def vec3(row: dict[str, str], a: str, b: str, c: str) -> Vec3 | None:
    values = tuple(num(row, name) for name in (a, b, c))
    return None if any(v is None for v in values) else values  # type: ignore[return-value]


def format_velocity(velocity: Vec3) -> str:
    return " ".join(f"{v:+7.4f}" for v in velocity)


@dataclass
class EncoderSpeed:
    """Rail speed over the latest two encoder samples, using the node's own clock."""

    value: float | None = None
    components: Vec3 | None = None
    interval_s: float | None = None
    frame_id: float | None = None
    reason: str = "waiting for encoders"
    _previous: tuple[float, float, Vec3] | None = field(default=None, repr=False)

    def extend(self, rows: list[dict[str, str]]) -> None:
        for row in rows:
            self.value = self.interval_s = None
            self.components = None
            previous, self._previous = self._previous, None
            stamp, frame = num(row, "t_encoder"), num(row, "frame_id")
            self.frame_id = frame
            position = tuple(num(row, f"enc_{a}") for a in "xyz")
            if stamp is None or frame is None or frame < 0 or any(v is None for v in position):
                self.reason = "invalid encoder sample"
                continue
            self._previous = (stamp, frame, position)  # type: ignore[assignment]
            if previous is None:
                self.reason = "need 2 samples"
                continue
            dt = stamp - previous[0]
            if (frame <= previous[1] or dt < 0 or not math.isfinite(dt)
                    or (dt == 0 and position != previous[2])):
                self._previous = None
                self.reason = "invalid sample timing"
                continue
            if dt == 0:
                # Consecutive polls can share a timestamp on the node's clock.
                self.reason = "need newer encoder time"
                continue
            components = tuple((p - q) / dt for p, q in zip(position, previous[2]))  # type: ignore[operator]
            speed = math.hypot(*components)
            if not math.isfinite(speed):
                self._previous = None
                self.reason = "invalid encoder speed"
                continue
            self.value, self.interval_s, self.reason = speed, dt, ""
            self.components = components  # type: ignore[assignment]


@dataclass
class MocapSpeed:
    """Relative speed of two valid, synchronized VRPN trackers in their common metre frame."""

    body: str = "Target"
    reference: str = "Chaser"
    value: float | None = None
    components: Vec3 | None = None
    interval_s: float | None = None
    frame_id: float | None = None
    reason: str = "waiting for mocap"
    _previous: tuple[float, tuple[float, float], Vec3] | None = field(default=None, repr=False)

    def extend(self, rows: list[dict[str, str]]) -> None:
        for row in rows:
            self.value = self.interval_s = None
            self.components = None
            previous, self._previous = self._previous, None
            self.frame_id = num(row, "frame_id")
            names = (self.body, self.reference)
            if any(f"{name}_valid" not in row for name in names):
                self.reason = "no mocap pair"
                continue
            invalid = next((name for name in names if num(row, f"{name}_valid") != 1), None)
            if invalid is not None:
                self.reason = f"{invalid} invalid"
                continue
            stamps = tuple(num(row, f"{name}_t_source") for name in names)
            frames = tuple(num(row, f"{name}_frames") for name in names)
            ages = tuple(num(row, f"{name}_age_s") for name in names)
            positions = [tuple(num(row, f"{name}_p{a}") for a in "xyz") for name in names]
            if (any(v is None for v in (*stamps, *frames, *ages, *positions[0], *positions[1]))
                    or any(v < 0 for v in (*frames, *ages))):  # type: ignore[operator]
                self.reason = "invalid mocap sample"
                continue
            # Pair only the same camera frame, allowing the VRPN timestamp's microsecond resolution.
            if abs(stamps[0] - stamps[1]) > 1e-6:  # type: ignore[operator]
                self.reason = "poses out of sync"
                continue
            relative = tuple(p - q for p, q in zip(*positions))  # type: ignore[operator]
            if not all(math.isfinite(v) for v in relative):
                self.reason = "invalid mocap sample"
                continue
            self._previous = (stamps[0], frames, relative)  # type: ignore[assignment]
            if previous is None:
                self.reason = "need 2 pose pairs"
                continue
            dt = stamps[0] - previous[0]  # type: ignore[operator]
            if (dt <= 0 or not math.isfinite(dt)
                    or any(f <= p for f, p in zip(frames, previous[1]))):  # type: ignore[operator]
                self._previous = None
                self.reason = "invalid mocap timing"
                continue
            components = tuple((p - q) / dt for p, q in zip(relative, previous[2]))
            speed = math.hypot(*components)
            if not math.isfinite(speed):
                self._previous = None
                self.reason = "invalid mocap speed"
                continue
            self.value, self.interval_s, self.reason = speed, dt, ""
            self.components = components  # type: ignore[assignment]


@dataclass
class Trajectory:
    rows: list[dict[str, str]] = field(default_factory=list)
    source: Path | None = None

    @property
    def run_rows(self) -> list[dict[str, str]]:
        return [r for r in self.rows if r.get("status") == "RUN"]

    @property
    def tracking_errors(self) -> list[float]:
        # Keep the valid measurement that triggered a tracking fault and the
        # final measurement recorded when an external run finishes normally.
        measured = (r for r in self.rows if r.get("status") in ("RUN", "COMPLETE", "STOPPED")
                    or r.get("reason") == "RAIL_TRACKING_ERROR")
        return [v for r in measured if (v := num(r, "track_err_m")) is not None]


def resolve_csv(path: Path) -> Path:
    return path / "trajectory.csv" if path.is_dir() else path


def load(path: Path) -> Trajectory:
    path = resolve_csv(path)
    with path.open(newline="", encoding="utf-8") as stream:
        rows = [r for r in csv.DictReader(stream) if r.get("k") and None not in r.values()]
    return Trajectory(rows, path)


class CsvTail:
    """Incremental reader for a CSV that is still being written; yields complete rows only."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.offset = 0
        self.header: list[str] | None = None
        self.pending = b""

    def poll(self) -> list[dict[str, str]]:
        try:
            with self.path.open("rb") as stream:
                stream.seek(self.offset)
                data = stream.read()
        except OSError:
            return []
        self.offset += len(data)
        *lines, self.pending = (self.pending + data).split(b"\n")
        rows = []
        for raw in lines:
            text = raw.decode("utf-8", "replace").rstrip("\r")
            if not text:
                continue
            cells = next(csv.reader([text]))
            if self.header is None:
                self.header = cells
            elif len(cells) == len(self.header):
                rows.append(dict(zip(self.header, cells)))
        return rows


@dataclass
class Series:
    x: list[float] = field(default_factory=list)
    y: list[float] = field(default_factory=list)
    z: list[float] = field(default_factory=list)
    distance: list[float] = field(default_factory=list)
    speed: list[float] = field(default_factory=list)
    velocity: list[Vec3] = field(default_factory=list)
    intervened: list[bool] = field(default_factory=list)

    def extend(self, rows: list[dict[str, str]]) -> None:
        for row in rows:
            h = [num(row, "hx"), num(row, "hy"), num(row, "hz")]
            if any(v is None for v in h):
                continue
            self.x.append(h[0])  # type: ignore[arg-type]
            self.y.append(h[1])  # type: ignore[arg-type]
            self.z.append(h[2])  # type: ignore[arg-type]
            self.distance.append(math.sqrt(sum(v * v for v in h)))  # type: ignore[operator]
            self.speed.append(norm3(row, "hvx", "hvy", "hvz") or 0.0)
            velocity = vec3(row, "hvx", "hvy", "hvz")
            if velocity is not None:
                self.velocity.append(velocity)
            self.intervened.append(row.get("intervened") == "1")


@dataclass
class Metric:
    group: str
    name: str
    value: str
    note: str = ""


def _ms(values: list[float]) -> float:
    return max(values) * 1e3


def stall_run(rows: list[dict[str, str]], threshold: float = 0.3, min_step_m: float = 0.003) -> tuple[int, int]:
    """Longest run of steps whose measured motion covered less than `threshold` of the commanded motion.

    Commanded motion = previous target - previous measurement; measured motion = this measurement -
    previous measurement. Steps commanding less than `min_step_m` are skipped. Returns (length, start k).
    """
    best = (0, -1)
    length, start = 0, -1
    for prev, cur in zip(rows, rows[1:]):
        cmd, act = [], []
        for axis in "xyz":
            p, t, c = num(prev, f"rail_{axis}"), num(prev, f"t{axis}"), num(cur, f"rail_{axis}")
            if p is None or t is None or c is None:
                break
            cmd.append(t - p)
            act.append(c - p)
        else:
            norm2 = sum(v * v for v in cmd)
            if norm2 < min_step_m * min_step_m:
                length, start = 0, -1
                continue
            progress = sum(a * b for a, b in zip(act, cmd)) / norm2
            if progress < threshold:
                if length == 0:
                    start = int(num(cur, "k") or 0)
                length += 1
                if length > best[0]:
                    best = (length, start)
            else:
                length, start = 0, -1
    return best


def frozen_run(rows: list[dict[str, str]]) -> tuple[int, int]:
    """Longest run of steps whose logged measurement repeats the previous one on all three axes.

    Logged at 1e-6 m, a live motion-capture stream practically never repeats; a repeat means a stale pose
    (for example a lost rigid body) or a stopped sensor. Returns (length, start k)."""
    best = (0, -1)
    length, start = 0, -1
    for prev, cur in zip(rows, rows[1:]):
        same = all(cur.get(f"rail_{a}") and cur.get(f"rail_{a}") == prev.get(f"rail_{a}") for a in "xyz")
        if same:
            if length == 0:
                start = int(num(cur, "k") or 0)
            length += 1
            if length > best[0]:
                best = (length, start)
        else:
            length, start = 0, -1
    return best


def summarize(trajectory: Trajectory, rail: RailConfig, ood_warn_z: float = 6.5) -> list[Metric]:
    rows, run = trajectory.rows, trajectory.run_rows
    if not rows:
        return []
    last = rows[-1]
    out = [Metric("Result", "Rows", str(len(rows)), f"{len(run)} RUN")]
    out.append(Metric("Result", "Final", f"{last.get('status', '?')} {last.get('reason', '')}".strip(), f"k = {last.get('k', '?')}"))
    t0, t1 = num(rows[0], "t_tick"), num(last, "t_tick")
    if t0 is not None and t1 is not None:
        out.append(Metric("Result", "Duration", f"{t1 - t0:.0f} s"))
    d, v = norm3(last, "hx", "hy", "hz"), vec3(last, "hvx", "hvy", "hvz")
    if d is not None:
        out.append(Metric("Result", "Final distance", f"{d:.3f} m",
                          f"Hill model vX/Y/Z {format_velocity(v)} m/s" if v is not None else ""))

    ages = [abs(v) for r in run if (v := num(r, "meas_age_s")) is not None]
    if ages:
        out.append(Metric("Timing", "Measurement age", f"{statistics.median(ages) * 1e3:.1f} ms median",
                          f"max {_ms(ages):.1f} ms, limit {rail.measurement_age_s * 1e3:.0f} ms"))
    jitter = [abs(v - rail.period_s) for r in run if (v := num(r, "dt_acq_s")) is not None]
    if jitter:
        out.append(Metric("Timing", "Sampling jitter", f"{_ms(jitter):.1f} ms max",
                          f"warning above {rail.sample_jitter_s * 1e3:.0f} ms"))
    wake = [w - t for r in run if (w := num(r, "t_wake")) is not None and (t := num(r, "t_tick")) is not None]
    if wake:
        out.append(Metric("Timing", "Wake lateness", f"{_ms(wake):.1f} ms max"))
    ctrl = [b - a for r in run if (a := num(r, "t_ctrl_start")) is not None and (b := num(r, "t_ctrl_end")) is not None]
    if ctrl:
        out.append(Metric("Timing", "Controller time", f"{_ms(ctrl):.2f} ms max"))
    ack = [b - a for r in run if (a := num(r, "t_sent")) is not None and (b := num(r, "t_plc_ack")) is not None]
    if ack:
        out.append(Metric("Timing", "PLC acknowledgment", f"{_ms(ack):.1f} ms max", "write returned, not motion"))

    track = trajectory.tracking_errors
    if track:
        out.append(Metric("Tracking", "Tracking error", f"{max(track) * 1e3:.1f} mm max",
                          f"limit {rail.tracking_error_m * 1e3:.0f} mm"))
    axis_err = [0.0, 0.0, 0.0]
    for prev, cur in zip(run, run[1:]):
        for i, axis in enumerate("xyz"):
            m, t = num(cur, f"rail_{axis}"), num(prev, f"t{axis}")
            if m is not None and t is not None:
                axis_err[i] = max(axis_err[i], abs(m - t))
    if len(run) > 1:
        out.append(Metric("Tracking", "Axis error X / Y / Z", " / ".join(f"{e * 1e3:.1f}" for e in axis_err) + " mm",
                          "measurement vs previous target"))
        length, start = stall_run(run)
        out.append(Metric("Tracking", "Stalled steps", str(length),
                          f"from k = {start}, progress < 30%" if length else "progress < 30% of command"))
        length, start = frozen_run(run)
        out.append(Metric("Tracking", "Repeated measurements", str(length),
                          f"from k = {start}, identical on all axes" if length else "identical on all axes"))

    for i, axis in enumerate("XYZ"):
        speeds = [abs(v) for r in run if (v := num(r, f"tv{axis.lower()}")) is not None]
        targets = [v for r in run if (v := num(r, f"t{axis.lower()}")) is not None]
        if speeds:
            peak, limit = max(speeds), rail.velocity_m_s[i]
            out.append(Metric("Commands", f"{axis} speed", f"{peak:.4f} m/s", f"{peak / limit * 100:.0f}% of {limit:g} m/s"))
        if targets:
            lo, hi = min(targets), max(targets)
            clearance = min(lo - rail.minimum_m[i], rail.maximum_m[i] - hi)
            out.append(Metric("Commands", f"{axis} target range", f"{lo:.4f} … {hi:.4f} m", f"clearance {clearance * 1e3:.0f} mm"))

    sigma = [v for r in run if (v := num(r, "obs_max_z")) is not None]
    if sigma:
        out.append(Metric("Controller", "Observation σ", f"{max(sigma):.2f} max",
                          f"{sum(1 for v in sigma if v > ood_warn_z)} steps above {ood_warn_z:g}"))
    if run and "intervened" in run[0]:
        interventions = sum(1 for r in run if r.get("intervened") == "1")
        out.append(Metric("Controller", "Safety interventions", str(interventions), f"{interventions / len(run) * 100:.0f}% of RUN steps"))
    if run and "backup" in run[0]:
        out.append(Metric("Controller", "Backup steps", str(sum(1 for r in run if r.get("backup") == "1"))))
    return out
