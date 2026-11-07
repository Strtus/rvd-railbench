"""Experiment directories written by rail_main (src/runtime/experiment.cpp) and simulation launch."""

from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from .config import read_snapshot
from .project import Project, ProjectError


@dataclass
class ExperimentRecord:
    path: Path
    started: str = ""
    outcome: str = "INCOMPLETE"
    reason: str = ""
    exit_code: int | None = None
    steps: int | None = None
    interventions: int | None = None
    mode: str = ""
    clock: str = ""
    backend: str = ""
    hardware: bool | None = None
    complete: bool = False

    @property
    def id(self) -> str:
        return self.path.name


def _json(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def read_experiment(path: Path) -> ExperimentRecord | None:
    metadata = _json(path / "metadata.json")
    if not metadata:
        return None
    record = ExperimentRecord(
        path=path,
        started=str(metadata.get("started_utc", "")),
        clock=str(metadata.get("clock", "")),
        backend=str(metadata.get("backend", "")),
        hardware=metadata.get("hardware"),
        complete=(path / "checksums.sha256").is_file(),
    )
    summary = _json(path / "summary.json")
    if summary:
        record.outcome = str(summary.get("outcome", "?"))
        record.reason = str(summary.get("reason", ""))
        record.exit_code = summary.get("exit_code")
        record.steps = summary.get("controller_steps")
        record.interventions = summary.get("interventions")
    snapshot = read_snapshot(path / "configuration.ini")
    if snapshot:
        record.mode = snapshot.simulation.mode
    return record


def list_experiments(directory: Path) -> list[ExperimentRecord]:
    if not directory.is_dir():
        return []
    records = [r for p in directory.iterdir() if p.is_dir() and (r := read_experiment(p)) is not None]
    records.sort(key=lambda r: (r.started, r.id), reverse=True)
    return records


def new_experiment_dir(directory: Path) -> Path:
    # Same naming as rail_main's default experiment directory.
    now = datetime.now(timezone.utc)
    return directory / f"{now:%Y%m%dT%H%M%S}-{now.microsecond:06d}Z"


def simulation_command(project: Project, override: Path, experiment_dir: Path) -> list[str]:
    """rail_main argument list for a simulated run. The TUI never passes --lab."""
    if project.build.rail_main is None:
        raise ProjectError("rail_main is not built (cmake --preset release && cmake --build --preset release)")
    args = [str(project.build.rail_main)]
    for path in project.profile.files:
        args += ["--config", str(path)]
    args += [
        "--config", str(override),
        "--model-dir", str(project.policy.directory),
        "--experiment-dir", str(experiment_dir),
        "--log", str(experiment_dir / "trajectory.csv"),
    ]
    return args
