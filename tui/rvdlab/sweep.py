"""Simulation sweeps: fit a Hill-to-rail mapping to a rail and check a rail profile."""

from __future__ import annotations

import asyncio
import math
import shutil
import tempfile
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from pathlib import Path

from .config import RailConfig, Vec3, envelope_violations, hill_to_rail, simulation_override
from .experiments import read_experiment
from .profiles import RailProfile, render
from .project import Project, ProjectError
from .telemetry import load, num

# Start set of the deployment checks: x = 100 m (top of the training range 50-100 m), small lateral offsets.
CANONICAL_STARTS: tuple[Vec3, ...] = tuple((100.0, y, z) for y in (-0.4, 0.0, 0.4) for z in (-0.4, 0.0, 0.4))

# Unconstrained identity mapping: the trajectory in Hill coordinates, independent of any rail.
REFERENCE = RailConfig(minimum_m=(-1e4, -1e4, -1e4), maximum_m=(1e4, 1e4, 1e4), velocity_m_s=(1e3, 1e3, 1e3),
                       acceleration_m_s2=(1e3, 1e3, 1e3), axis=(0, 1, 2), sign=(1.0, 1.0, 1.0),
                       scale=(1.0, 1.0, 1.0), target_m=(0.0, 0.0, 0.0))


@dataclass
class RunResult:
    start: Vec3
    outcome: str
    reason: str
    steps: int
    rows: list[dict[str, str]] = field(default_factory=list)


def inside(rail: RailConfig, start: Vec3) -> bool:
    return not envelope_violations(rail, hill_to_rail(rail, start))


def validation_starts(rail: RailConfig) -> list[Vec3]:
    """Canonical starts, pulled in along x when x = 100 m lies at or beyond the end of the travel."""
    a, s, k = rail.axis[0], rail.sign[0], rail.scale[0]
    ends = sorted(((rail.minimum_m[a] - rail.target_m[a]) / (s * k), (rail.maximum_m[a] - rail.target_m[a]) / (s * k)))
    x = min(100.0, 0.999 * ends[1])
    return [(x, y, z) for _, y, z in CANONICAL_STARTS if inside(rail, (x, y, z))]


async def run_sweep(project: Project, profile: RailProfile, starts: Sequence[Vec3],
                    progress: Callable[[int, int], None] | None = None, parallel: int = 4) -> list[RunResult]:
    if project.build.rail_main is None:
        raise ProjectError("rail_main is not built")
    workdir = Path(tempfile.mkdtemp(prefix="rvdlab-sweep-"))
    try:
        profile_file = workdir / "profile.ini"
        profile_file.write_text(render(profile), encoding="utf-8")
        base = [project.root / "configs" / "pfcr_dqp.ini", project.root / "configs" / "simulation.ini", profile_file]
        gate = asyncio.Semaphore(parallel)
        done = 0

        async def one(index: int, start: Vec3) -> RunResult:
            nonlocal done
            override = workdir / f"start-{index}.ini"
            override.write_text(simulation_override(start, "ideal", True, -1, 12345), encoding="utf-8")
            experiment = workdir / f"run-{index}"
            args = [str(project.build.rail_main)]
            for path in (*base, override):
                args += ["--config", str(path)]
            args += ["--model-dir", str(project.policy.directory), "--experiment-dir", str(experiment),
                     "--log", str(experiment / "trajectory.csv")]
            async with gate:
                process = await asyncio.create_subprocess_exec(
                    *args, cwd=str(project.root), stdin=asyncio.subprocess.DEVNULL,
                    stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL)
                try:
                    await asyncio.wait_for(process.wait(), timeout=120)
                except asyncio.TimeoutError:
                    process.kill()
                    await process.wait()
            record = read_experiment(experiment)
            rows = load(experiment).rows if (experiment / "trajectory.csv").is_file() else []
            done += 1
            if progress:
                progress(done, len(starts))
            if record is None:
                return RunResult(start, "NOT STARTED", f"exit code {process.returncode}", 0, rows)
            return RunResult(start, record.outcome, record.reason, record.steps or 0, rows)

        return list(await asyncio.gather(*(one(i, s) for i, s in enumerate(starts))))
    finally:
        shutil.rmtree(workdir, ignore_errors=True)


def _round_down(value: float, digits: int = 3) -> float:
    if not (value > 0 and math.isfinite(value)):
        return value
    exponent = math.floor(math.log10(value)) - (digits - 1)
    return round(math.floor(value / 10.0**exponent + 1e-9) * 10.0**exponent, max(0, -exponent))


@dataclass
class Fit:
    scale: Vec3
    target: Vec3
    limited_by: tuple[str, str, str]
    extent: tuple[tuple[float, float], tuple[float, float], tuple[float, float]]
    peak: Vec3


def fit_mapping(rail: RailConfig, reference: Sequence[RunResult], margin: float = 0.1) -> Fit:
    """Largest scale per Hill axis that keeps the reference trajectories inside the travel and the speed
    limit, both reduced by `margin`; the docking port is placed so the trajectories are centred."""
    extent = [[0.0, 0.0] for _ in range(3)]  # includes the docking port at the origin
    peak = [0.0, 0.0, 0.0]
    for result in reference:
        for row in result.rows:
            for j, name in enumerate(("hx", "hy", "hz")):
                value = num(row, name)
                if value is not None:
                    extent[j] = [min(extent[j][0], value), max(extent[j][1], value)]
            if row.get("status") == "RUN":
                for j, name in enumerate(("tvx", "tvy", "tvz")):
                    value = num(row, name)
                    if value is not None:
                        peak[j] = max(peak[j], abs(value))
    scale, limited, target = [0.0, 0.0, 0.0], [], list(rail.target_m)
    for j in range(3):
        a = rail.axis[j]
        by_travel = (rail.maximum_m[a] - rail.minimum_m[a]) * (1 - margin) / max(extent[j][1] - extent[j][0], 1e-6)
        by_speed = rail.velocity_m_s[a] * (1 - margin) / peak[j] if peak[j] > 0 else math.inf
        scale[j] = _round_down(min(by_travel, by_speed))
        limited.append("travel" if by_travel <= by_speed else "speed")
        centre = (rail.minimum_m[a] + rail.maximum_m[a]) / 2
        target[a] = round(centre - rail.sign[j] * scale[j] * (extent[j][0] + extent[j][1]) / 2, 4)
    return Fit((scale[0], scale[1], scale[2]), (target[0], target[1], target[2]),
               (limited[0], limited[1], limited[2]),
               ((extent[0][0], extent[0][1]), (extent[1][0], extent[1][1]), (extent[2][0], extent[2][1])),
               (peak[0], peak[1], peak[2]))


@dataclass
class Check:
    start: Vec3
    outcome: str
    reason: str
    steps: int
    speed_use: Vec3
    clearance_m: float


def evaluate(rail: RailConfig, result: RunResult) -> Check:
    use, clearance = [0.0, 0.0, 0.0], math.inf
    for row in result.rows:
        if row.get("status") != "RUN":
            continue
        for i, axis in enumerate("xyz"):
            speed, target = num(row, f"tv{axis}"), num(row, f"t{axis}")
            if speed is not None:
                use[i] = max(use[i], abs(speed) / rail.velocity_m_s[i])
            if target is not None:
                clearance = min(clearance, target - rail.minimum_m[i], rail.maximum_m[i] - target)
    return Check(result.start, result.outcome, result.reason, result.steps, (use[0], use[1], use[2]), clearance)
