"""Reader for the rail_main INI schema (src/runtime/config.cpp) and the rail/Hill frame map."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

Vec3 = tuple[float, float, float]
MODEL_ID = "E_PFCR-DQP_s41_step5120000"
DEFAULT_FILES = ("configs/rail.ini", "configs/pfcr_dqp.ini", "configs/simulation.ini")
SECTIONS = {"rail", "mapping", "controller", "simulation", "experiment"}


class ConfigError(ValueError):
    pass


# Defaults mirror the compiled compatibility defaults of rail_main.
@dataclass
class RailConfig:
    period_s: float = 1.0
    axis: tuple[int, int, int] = (0, 1, 2)
    sign: Vec3 = (1.0, 1.0, 1.0)
    scale: Vec3 = (1.0, 0.1, 0.1)
    target_m: Vec3 = (0.0, 0.0, 0.0)
    minimum_m: Vec3 = (0.0, -0.5, -0.5)
    maximum_m: Vec3 = (20.0, 0.5, 0.5)
    velocity_m_s: Vec3 = (0.5, 0.2, 0.2)
    acceleration_m_s2: Vec3 = (0.5, 0.2, 0.2)
    measurement_age_s: float = 0.2
    measurement_late_s: float = 0.3
    clock_skew_s: float = 0.005
    sample_jitter_s: float = 0.2
    communication_timeout_s: float = 0.3
    tracking_error_m: float = 0.05
    sensor_disagreement_m: float = 0.05
    compensate_latency: bool = False
    command_mode: str = "arrive"
    lead_s: float = 0.5
    tracking_gain_per_s: float = 0.5


@dataclass
class ControllerConfig:
    type: str = "pfcr-dqp"
    model_id: str = MODEL_ID
    backend: str = "builtin"
    ood_warn_z: float = 6.5


@dataclass
class SimulationConfig:
    mode: str = "ideal"
    seed: int = 12345
    noise_m: float = 0.0005
    latency_s: float = 0.010
    bias_m: Vec3 = (0.0, 0.0, 0.0)
    freeze_at: int = -1
    repeat_pose_at: int = -1
    fail_read_at: int = -1
    fail_send_at: int = -1
    encoder: bool = True
    plc_acceleration_m_s2: Vec3 = (0.2, 0.2, 0.2)
    plc_delay_s: float = 0.03
    start_hill_m: Vec3 = (18.0, 0.3, -0.2)
    fast: bool = False
    open_loop: bool = False
    max_steps: int = -1


@dataclass
class Profile:
    files: list[Path] = field(default_factory=list)
    rail: RailConfig = field(default_factory=RailConfig)
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    experiment_dir: str | None = None


def _number(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise ConfigError(f"expected finite number: {text}") from None
    if not math.isfinite(value):
        raise ConfigError(f"expected finite number: {text}")
    return value


def _integer(text: str) -> int:
    value = _number(text)
    if value != math.floor(value) or value < -1:
        raise ConfigError(f"invalid integer: {text}")
    return int(value)


def _boolean(text: str) -> bool:
    if text in ("true", "false"):
        return text == "true"
    raise ConfigError(f"expected true or false: {text}")


def _triple(text: str) -> Vec3:
    parts = text.split(",")
    if len(parts) != 3:
        raise ConfigError("expected exactly three comma-separated values")
    return (_number(parts[0].strip()), _number(parts[1].strip()), _number(parts[2].strip()))


_RAIL_TRIPLES = {
    "rail.minimum_m": "minimum_m",
    "rail.maximum_m": "maximum_m",
    "rail.velocity_m_s": "velocity_m_s",
    "rail.acceleration_m_s2": "acceleration_m_s2",
    "mapping.sign": "sign",
    "mapping.scale": "scale",
    "mapping.target_m": "target_m",
}
_RAIL_NUMBERS = {
    "rail.measurement_age_s": "measurement_age_s",
    "rail.measurement_late_s": "measurement_late_s",
    "rail.clock_skew_s": "clock_skew_s",
    "rail.sample_jitter_s": "sample_jitter_s",
    "rail.communication_timeout_s": "communication_timeout_s",
    "rail.tracking_error_m": "tracking_error_m",
    "rail.sensor_disagreement_m": "sensor_disagreement_m",
    "rail.lead_s": "lead_s",
    "rail.tracking_gain_per_s": "tracking_gain_per_s",
    "controller.period_s": "period_s",
}
COMMAND_MODES = ("arrive", "lead")
SIMULATION_MODES = ("ideal", "bounded", "plc")
_SIM_NUMBERS = {"simulation.noise_m": "noise_m", "simulation.latency_s": "latency_s",
                "simulation.plc_delay_s": "plc_delay_s"}
_SIM_INTEGERS = {
    "simulation.freeze_at": "freeze_at",
    "simulation.repeat_pose_at": "repeat_pose_at",
    "simulation.fail_read_at": "fail_read_at",
    "simulation.fail_send_at": "fail_send_at",
    "simulation.max_steps": "max_steps",
}
_SIM_BOOLEANS = {"simulation.fast": "fast", "simulation.open_loop": "open_loop", "simulation.encoder": "encoder"}
_SIM_TRIPLES = {"simulation.bias_m": "bias_m", "simulation.start_hill_m": "start_hill_m",
                "simulation.plc_acceleration_m_s2": "plc_acceleration_m_s2"}


def _apply(profile: Profile, key: str, value: str) -> None:
    rail, ctrl, sim = profile.rail, profile.controller, profile.simulation
    if key in _RAIL_TRIPLES:
        setattr(rail, _RAIL_TRIPLES[key], _triple(value))
    elif key in _RAIL_NUMBERS:
        setattr(rail, _RAIL_NUMBERS[key], _number(value))
    elif key == "rail.compensate_latency":
        rail.compensate_latency = _boolean(value)
    elif key == "rail.command_mode":
        if value not in COMMAND_MODES:
            raise ConfigError("command mode must be arrive or lead")
        rail.command_mode = value
    elif key == "mapping.axis":
        axes = _triple(value)
        if any(a not in (0.0, 1.0, 2.0) for a in axes):
            raise ConfigError("axis must be 0, 1 or 2")
        rail.axis = (int(axes[0]), int(axes[1]), int(axes[2]))
    elif key == "controller.type":
        if value != "pfcr-dqp":
            raise ConfigError("unsupported controller")
        ctrl.type = value
    elif key == "controller.model_id":
        if value != MODEL_ID:
            raise ConfigError("incompatible policy artifact")
        ctrl.model_id = value
    elif key == "controller.backend":
        if value not in ("builtin", "onnx"):
            raise ConfigError("unsupported policy backend")
        ctrl.backend = value
    elif key == "controller.ood_warn_z":
        ctrl.ood_warn_z = _number(value)
    elif key == "simulation.mode":
        if value not in SIMULATION_MODES:
            raise ConfigError("unsupported simulation mode")
        sim.mode = value
    elif key == "simulation.seed":
        seed = _number(value)
        if seed < 0 or seed > 2**32 - 1 or seed != math.floor(seed):
            raise ConfigError("seed must be uint32")
        sim.seed = int(seed)
    elif key in _SIM_NUMBERS:
        setattr(sim, _SIM_NUMBERS[key], _number(value))
    elif key in _SIM_INTEGERS:
        setattr(sim, _SIM_INTEGERS[key], _integer(value))
    elif key in _SIM_BOOLEANS:
        setattr(sim, _SIM_BOOLEANS[key], _boolean(value))
    elif key in _SIM_TRIPLES:
        setattr(sim, _SIM_TRIPLES[key], _triple(value))
    elif key == "experiment.directory":
        profile.experiment_dir = value
    else:
        raise ConfigError(f"unknown or immutable configuration key: {key}")


def apply_file(profile: Profile, path: Path) -> None:
    seen: set[str] = set()
    section = ""
    for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line or line[0] in "#;":
            continue
        try:
            if line.startswith("[") and line.endswith("]"):
                section = line[1:-1]
                if section not in SECTIONS:
                    raise ConfigError("unknown configuration section")
                continue
            if "=" not in line:
                raise ConfigError("expected key = value")
            name, value = (part.strip() for part in line.split("=", 1))
            if not value:
                raise ConfigError("empty configuration value")
            key = f"{section}.{name}"
            if key in seen:
                raise ConfigError("duplicate configuration key")
            seen.add(key)
            _apply(profile, key, value)
        except ConfigError as error:
            raise ConfigError(f"{path}:{lineno}: {error}") from None
    profile.files.append(path)


def validate(profile: Profile) -> None:
    r = profile.rail
    if r.period_s != 1.0:
        raise ConfigError("PFCR artifact requires exactly 1 s control period")
    if sorted(r.axis) != [0, 1, 2]:
        raise ConfigError("mapping axes must be a permutation of 0,1,2")
    for i in range(3):
        if r.sign[i] not in (1.0, -1.0):
            raise ConfigError("mapping sign must be +1 or -1")
        if not r.scale[i] > 0:
            raise ConfigError("mapping scales must be finite and positive")
        if not r.minimum_m[i] < r.maximum_m[i]:
            raise ConfigError("rail minimum must be below maximum")
        if not (r.velocity_m_s[i] > 0 and r.acceleration_m_s2[i] > 0):
            raise ConfigError("rail velocity and acceleration limits must be positive")
    if not 0 < r.lead_s <= 5:
        raise ConfigError("lead_s must be in (0, 5] s")
    if not 0 <= r.tracking_gain_per_s * r.period_s <= 1:
        raise ConfigError("tracking_gain_per_s must be in [0, 1/period_s]")
    if not r.sensor_disagreement_m > 0:
        raise ConfigError("sensor_disagreement_m must be positive")


def load_profile(root: Path, files: list[Path] | None = None) -> Profile:
    profile = Profile()
    for path in files or [root / name for name in DEFAULT_FILES]:
        apply_file(profile, path)
    validate(profile)
    return profile


def read_snapshot(path: Path) -> Profile | None:
    """Profile from an experiment's configuration.ini, or None when unreadable."""
    profile = Profile()
    try:
        apply_file(profile, path)
    except (ConfigError, OSError):
        return None
    return profile


def hill_to_rail(rail: RailConfig, hill: Vec3) -> Vec3:
    out = [0.0, 0.0, 0.0]
    for j in range(3):
        a = rail.axis[j]
        out[a] = rail.target_m[a] + rail.sign[j] * rail.scale[j] * hill[j]
    return (out[0], out[1], out[2])


def rail_to_hill(rail: RailConfig, position: Vec3) -> Vec3:
    h = [rail.sign[j] * (position[rail.axis[j]] - rail.target_m[rail.axis[j]]) / rail.scale[j] for j in range(3)]
    return (h[0], h[1], h[2])


def envelope_violations(rail: RailConfig, position: Vec3) -> list[str]:
    out = []
    for i, name in enumerate("XYZ"):
        if position[i] < rail.minimum_m[i]:
            out.append(f"{name} below minimum by {(rail.minimum_m[i] - position[i]) * 1e3:.1f} mm")
        elif position[i] > rail.maximum_m[i]:
            out.append(f"{name} above maximum by {(position[i] - rail.maximum_m[i]) * 1e3:.1f} mm")
    return out


def simulation_override(start: Vec3, mode: str, fast: bool, max_steps: int, seed: int, *,
                        freeze_at: int = -1, repeat_pose_at: int = -1, fail_read_at: int = -1,
                        fail_send_at: int = -1) -> str:
    """Last --config file of a TUI run: rail_main has no CLI switch back to ideal or real time."""
    return (
        "[simulation]\n"
        f"mode = {mode}\n"
        f"fast = {'true' if fast else 'false'}\n"
        f"max_steps = {max_steps}\n"
        f"seed = {seed}\n"
        f"start_hill_m = {start[0]!r},{start[1]!r},{start[2]!r}\n"
        f"freeze_at = {freeze_at}\n"
        f"repeat_pose_at = {repeat_pose_at}\n"
        f"fail_read_at = {fail_read_at}\n"
        f"fail_send_at = {fail_send_at}\n"
    )
