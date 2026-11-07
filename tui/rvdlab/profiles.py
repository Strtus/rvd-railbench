"""Rail profiles: a user's rail and its Hill mapping, stored as rail_main INI files."""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

from .config import ConfigError, Profile, RailConfig, Vec3, apply_file, validate

ACTIVE = "active"
_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")


@dataclass
class RailProfile:
    name: str
    rail: RailConfig
    start_hill_m: Vec3


def profile_path(directory: Path, name: str) -> Path:
    return directory / f"{name}.ini"


def list_profiles(directory: Path) -> list[str]:
    return sorted(p.stem for p in directory.glob("*.ini")) if directory.is_dir() else []


def read_active(directory: Path) -> str | None:
    try:
        name = (directory / ACTIVE).read_text(encoding="utf-8").strip()
    except OSError:
        return None
    return name if name and profile_path(directory, name).is_file() else None


def set_active(directory: Path, name: str) -> None:
    (directory / ACTIVE).write_text(name + "\n", encoding="utf-8")


def load_rail_profile(path: Path) -> RailProfile:
    profile = Profile()
    apply_file(profile, path)
    validate(profile)
    return RailProfile(path.stem, profile.rail, profile.simulation.start_hill_m)


def _triple(values: tuple[float, ...]) -> str:
    return ",".join(repr(float(v)) for v in values)


def render(profile: RailProfile, notes: tuple[str, ...] = ()) -> str:
    r = profile.rail
    lines = [f"# RVDLAB rail profile: {profile.name}", *(f"# {note}" for note in notes),
             "[rail]",
             f"minimum_m = {_triple(r.minimum_m)}",
             f"maximum_m = {_triple(r.maximum_m)}",
             f"velocity_m_s = {_triple(r.velocity_m_s)}",
             f"acceleration_m_s2 = {_triple(r.acceleration_m_s2)}",
             f"measurement_age_s = {r.measurement_age_s!r}",
             f"measurement_late_s = {r.measurement_late_s!r}",
             f"clock_skew_s = {r.clock_skew_s!r}",
             f"sample_jitter_s = {r.sample_jitter_s!r}",
             f"communication_timeout_s = {r.communication_timeout_s!r}",
             f"tracking_error_m = {r.tracking_error_m!r}",
             f"compensate_latency = {'true' if r.compensate_latency else 'false'}",
             "[mapping]",
             f"axis = {','.join(str(a) for a in r.axis)}",
             f"sign = {','.join(str(int(s)) for s in r.sign)}",
             f"scale = {_triple(r.scale)}",
             f"target_m = {_triple(r.target_m)}",
             "[simulation]",
             f"start_hill_m = {_triple(profile.start_hill_m)}"]
    return "\n".join(lines) + "\n"


def check(profile: RailProfile) -> None:
    if not _NAME.match(profile.name):
        raise ConfigError("profile name may use letters, digits, '-', '_' and '.'")
    candidate = Profile(rail=profile.rail)
    candidate.simulation.start_hill_m = profile.start_hill_m
    validate(candidate)


def save(directory: Path, profile: RailProfile, notes: tuple[str, ...] = ()) -> Path:
    check(profile)
    directory.mkdir(parents=True, exist_ok=True)
    path = profile_path(directory, profile.name)
    path.write_text(render(profile, notes), encoding="utf-8")
    return path
