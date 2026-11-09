"""Hardware profiles: the axes a node exposes and how its driver reaches them."""

from __future__ import annotations

import copy
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

AXES = ("x", "y", "z", "pitch", "yaw", "roll")
LINEAR = ("x", "y", "z")
TIMINGS = ("poll_s", "stall_timeout_s", "enable_timeout_s", "stop_timeout_s")

SIMULATOR: dict[str, Any] = {
    "driver": "simulator",
    "poll_s": 0.1,
    "axes": {
        "x": {"minimum": 0.0, "maximum": 20.0, "max_speed": 0.5, "acceleration": 0.5, "tolerance": 0.001},
        "y": {"minimum": -0.5, "maximum": 0.5, "max_speed": 0.2, "acceleration": 0.2, "tolerance": 0.001},
        "z": {"minimum": -0.5, "maximum": 0.5, "max_speed": 0.2, "acceleration": 0.2, "tolerance": 0.001},
    },
}


class ProfileError(ValueError):
    pass


@dataclass(frozen=True)
class Axis:
    name: str
    minimum: float
    maximum: float
    max_speed: float
    tolerance: float
    resolution: float
    motion: bool

    @property
    def unit(self) -> str:
        return "m" if self.name in LINEAR else "rad"

    @property
    def min_speed(self) -> float:
        return self.resolution  # one count per second

    def describe(self) -> dict[str, Any]:
        return {
            "unit": self.unit,
            "minimum": self.minimum,
            "maximum": self.maximum,
            "max_speed": self.max_speed,
            "min_speed": self.min_speed,
            "tolerance": self.tolerance,
            "resolution": self.resolution,
            "motion": self.motion,
        }


def _finite(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ProfileError(f"{where} must be a finite number")
    return float(value)


def _positive(value: Any, where: str) -> float:
    if _finite(value, where) <= 0:
        raise ProfileError(f"{where} must be positive")
    return float(value)


def _integer(value: Any, where: str) -> int:
    if type(value) is not int:
        raise ProfileError(f"{where} must be an integer")
    return value


def _object(value: Any, where: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ProfileError(f"{where} must be an object")
    return value


def _check(value: Any, where: str) -> None:
    check = _object(value, where)
    if set(check) != {"d", "value"}:
        raise ProfileError(f"{where} needs exactly d and value")
    _integer(check["d"], f"{where}.d")
    _integer(check["value"], f"{where}.value")


def _modbus(profile: dict[str, Any]) -> None:
    bus = _object(profile.get("modbus"), "modbus")
    if not isinstance(bus.get("host"), str) or not bus["host"]:
        raise ProfileError("modbus.host is required")
    for key in ("d_base", "m_base"):
        _integer(bus.get(key), f"modbus.{key}")
    for key in ("port", "unit", "d_stride"):
        if key in bus:
            _integer(bus[key], f"modbus.{key}")
    if "timeout_s" in bus:
        _positive(bus["timeout_s"], "modbus.timeout_s")
    if bus.get("word_order", "low_first") not in ("low_first", "high_first"):
        raise ProfileError("modbus.word_order must be low_first or high_first")
    interlock = _object(profile.get("interlock", {}), "interlock")
    if set(interlock) - {"identity", "emergency_m"}:
        raise ProfileError("interlock accepts identity and emergency_m")
    if "identity" in interlock:
        _check(interlock["identity"], "interlock.identity")
    if "emergency_m" in interlock:
        _integer(interlock["emergency_m"], "interlock.emergency_m")
    for name, axis in profile["axes"].items():
        kind = axis.get("type")
        if kind not in ("pulse", "servo"):
            raise ProfileError(f"{name}.type must be pulse or servo")
        bits = ("start_m", "stop_m") if kind == "pulse" else ("enable_m", "stop_m", "enabled_d", "closed_d")
        for key in ("target_d", "speed_d", "encoder_d", "zero", "min_count", "max_count", *bits):
            _integer(axis.get(key), f"{name}.{key}")
        for key in ("quickstop_m", "pending_d", "sample_count_d"):
            if key in axis:
                _integer(axis[key], f"{name}.{key}")
        if axis["min_count"] >= axis["max_count"]:
            raise ProfileError(f"{name}: min_count must be below max_count")
        if _finite(axis.get("scale"), f"{name}.scale") == 0:
            raise ProfileError(f"{name}.scale must not be zero")
        ready = axis.get("ready", [])
        if not isinstance(ready, list):
            raise ProfileError(f"{name}.ready must be a list")
        for i, check in enumerate(ready):
            _check(check, f"{name}.ready[{i}]")


def _simulator(profile: dict[str, Any]) -> None:
    for name, axis in profile["axes"].items():
        lo = _finite(axis.get("minimum"), f"{name}.minimum")
        hi = _finite(axis.get("maximum"), f"{name}.maximum")
        if lo >= hi:
            raise ProfileError(f"{name}: minimum must be below maximum")
        for key in ("acceleration", "resolution"):
            if key in axis:
                _positive(axis[key], f"{name}.{key}")
        if "start" in axis and not lo <= _finite(axis["start"], f"{name}.start") <= hi:
            raise ProfileError(f"{name}.start is outside the limits")


def validate(profile: Any) -> dict[str, Any]:
    profile = copy.deepcopy(_object(profile, "profile"))
    if profile.get("driver") not in ("simulator", "modbus"):
        raise ProfileError("driver must be simulator or modbus")
    axes = _object(profile.get("axes"), "axes")
    if not axes or set(axes) - set(AXES):
        raise ProfileError(f"axes must be a non-empty subset of {', '.join(AXES)}")
    for name, axis in axes.items():
        _object(axis, name)
        _positive(axis.get("max_speed"), f"{name}.max_speed")
        _positive(axis.get("tolerance"), f"{name}.tolerance")
        if not isinstance(axis.get("motion", True), bool):
            raise ProfileError(f"{name}.motion must be true or false")
    for key in TIMINGS:
        if key in profile:
            _positive(profile[key], key)
    (_modbus if profile["driver"] == "modbus" else _simulator)(profile)
    return profile


def load(path: str | Path) -> dict[str, Any]:
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    except (OSError, ValueError) as exc:
        raise ProfileError(f"cannot read profile {path}: {exc}") from None
    return validate(data)


def axes(profile: dict[str, Any]) -> dict[str, Axis]:
    result = {}
    for name in AXES:
        spec = profile["axes"].get(name)
        if spec is None:
            continue
        if profile["driver"] == "modbus":
            scale = float(spec["scale"])
            ends = sorted(((spec["min_count"] - spec["zero"]) * scale, (spec["max_count"] - spec["zero"]) * scale))
            lo, hi, resolution = ends[0], ends[1], abs(scale)
        else:
            lo, hi, resolution = float(spec["minimum"]), float(spec["maximum"]), float(spec.get("resolution", 1e-6))
        result[name] = Axis(name, lo, hi, float(spec["max_speed"]), float(spec["tolerance"]), resolution,
                            spec.get("motion", True))
    return result
