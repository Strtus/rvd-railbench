"""Read-only view of the checkout: version, build, git, policy artifact and device status."""

from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

from .config import DEFAULT_FILES, ConfigError, Profile, load_profile
from .profiles import profile_path, read_active

EXE = ".exe" if os.name == "nt" else ""
BUILD_DIRS = ("build/release", "build/release/Release", "build/dev", "build/minimal")
_CACHE_KEYS = {
    "RVD_SIMULATION_ONLY",
    "RVD_ENABLE_ONNX",
    "CMAKE_BUILD_TYPE",
    "CMAKE_CXX_COMPILER_ID",
    "CMAKE_CXX_COMPILER_VERSION",
}


class ProjectError(RuntimeError):
    pass


def _is_root(path: Path) -> bool:
    cmake = path / "CMakeLists.txt"
    return cmake.is_file() and "project(rvd_railbench" in cmake.read_text(encoding="utf-8", errors="replace")


def find_root(start: Path | None = None) -> Path:
    bases = [Path(start).resolve()] if start else [Path.cwd().resolve(), Path(__file__).resolve().parent]
    for base in bases:
        for path in (base, *base.parents):
            if _is_root(path):
                return path
    raise ProjectError("RVD-RailBench checkout not found; pass --root")


def read_version(root: Path) -> str:
    text = (root / "CMakeLists.txt").read_text(encoding="utf-8", errors="replace")
    match = re.search(r"project\(\s*rvd_railbench\s+VERSION\s+([0-9][0-9.]*)", text)
    return match.group(1) if match else "unknown"


def _on(value: str | None) -> bool:
    return (value or "").upper() in ("ON", "TRUE", "1", "YES")


@dataclass
class Build:
    directory: Path | None = None
    rail_main: Path | None = None
    rail_replay: Path | None = None
    cache: dict[str, str] = field(default_factory=dict)

    @property
    def simulation_only(self) -> bool | None:
        value = self.cache.get("RVD_SIMULATION_ONLY")
        return None if value is None else _on(value)

    @property
    def onnx(self) -> bool:
        return _on(self.cache.get("RVD_ENABLE_ONNX"))

    @property
    def build_type(self) -> str:
        return self.cache.get("CMAKE_BUILD_TYPE", "")


def read_cache(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    if path.is_file():
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            if line.startswith(("#", "//")) or ":" not in line or "=" not in line:
                continue
            name, rest = line.split(":", 1)
            if name in _CACHE_KEYS:
                out[name] = rest.split("=", 1)[1]
    return out


def find_build(root: Path) -> Build:
    candidates = [Path(os.environ["RVD_BUILD_DIR"])] if os.environ.get("RVD_BUILD_DIR") else []
    candidates += [root / name for name in BUILD_DIRS]
    for directory in candidates:
        rail_main = directory / f"rail_main{EXE}"
        if rail_main.is_file():
            replay = directory / f"rail_replay{EXE}"
            return Build(directory, rail_main, replay if replay.is_file() else None,
                         read_cache(directory / "CMakeCache.txt"))
    return Build()


@dataclass
class Git:
    branch: str = ""
    commit: str = ""
    dirty: bool | None = None


def read_git(root: Path) -> Git:
    def git(*args: str) -> str | None:
        try:
            result = subprocess.run(["git", "-C", str(root), *args], capture_output=True, text=True, timeout=3)
        except (OSError, subprocess.TimeoutExpired):
            return None
        return result.stdout.strip() if result.returncode == 0 else None

    status = git("status", "--porcelain")
    return Git(git("rev-parse", "--abbrev-ref", "HEAD") or "", git("rev-parse", "--short", "HEAD") or "",
               None if status is None else bool(status))


@dataclass
class Artifact:
    name: str
    expected: str
    actual: str | None

    @property
    def ok(self) -> bool:
        return self.actual == self.expected


@dataclass
class Policy:
    model_id: str
    directory: Path
    manifest: dict
    artifacts: list[Artifact]

    @property
    def verified(self) -> bool:
        return bool(self.artifacts) and all(a.ok for a in self.artifacts)

    @property
    def missing(self) -> list[Artifact]:
        return [a for a in self.artifacts if a.actual is None]


def read_policy(root: Path, model_id: str) -> Policy:
    directory = root / "policies" / model_id
    path = directory / "manifest.json"
    try:
        manifest = json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}
    except (OSError, ValueError):
        manifest = {}
    artifacts = []
    for name, digest in manifest.get("artifacts", {}).items():
        file = directory / name
        actual = hashlib.sha256(file.read_bytes()).hexdigest() if file.is_file() else None
        artifacts.append(Artifact(name, digest, actual))
    return Policy(model_id, directory, manifest, artifacts)


@dataclass
class Status:
    label: str
    ok: bool
    text: str
    detail: str = ""


@dataclass
class Project:
    root: Path
    version: str
    build: Build
    git: Git
    policy: Policy
    profile: Profile
    experiments_dir: Path
    profiles_dir: Path
    rail_profile: str | None = None  # active rail profile name
    explicit_configs: bool = False  # --config given: profiles are bypassed
    config_error: str | None = None

    def relative(self, path: Path | None) -> str:
        if path is None:
            return ""
        try:
            return path.resolve().relative_to(self.root).as_posix()
        except ValueError:
            return str(path)


def config_files(root: Path, profiles_dir: Path, configs: list[Path] | None) -> tuple[list[Path], str | None]:
    """Explicit --config files, else the controller and simulation files plus the active rail profile,
    else the checked-in example files."""
    if configs:
        return list(configs), None
    name = read_active(profiles_dir)
    if name:
        return [root / "configs" / "pfcr_dqp.ini", root / "configs" / "simulation.ini",
                profile_path(profiles_dir, name)], name
    return [root / name for name in DEFAULT_FILES], None


def load_project(root: Path | None = None, configs: list[Path] | None = None,
                 experiments: Path | None = None, profiles: Path | None = None) -> Project:
    base = find_root(root)
    if profiles is None and os.environ.get("RVDLAB_PROFILES"):
        profiles = Path(os.environ["RVDLAB_PROFILES"])
    if experiments is None and os.environ.get("RVDLAB_EXPERIMENTS"):
        experiments = Path(os.environ["RVDLAB_EXPERIMENTS"])
    profiles_dir = (profiles or base / "profiles").resolve()
    files, rail_profile = config_files(base, profiles_dir, configs)
    try:
        profile, error = load_profile(base, files), None
    except (ConfigError, OSError) as exc:
        profile, error = Profile(), str(exc)
    return Project(
        root=base,
        version=read_version(base),
        build=find_build(base),
        git=read_git(base),
        policy=read_policy(base, profile.controller.model_id),
        profile=profile,
        experiments_dir=(experiments or base / "experiments").resolve(),
        profiles_dir=profiles_dir,
        rail_profile=rail_profile,
        explicit_configs=bool(configs),
        config_error=error,
    )


def system_status(project: Project) -> list[Status]:
    """Status lines for the splash. Static checks only; never opens a device connection."""
    build, policy = project.build, project.policy
    if project.explicit_configs:
        rail = Status("Rail Profile", True, "Custom configuration", ", ".join(f.name for f in project.profile.files))
    elif project.rail_profile:
        rail = Status("Rail Profile", True, project.rail_profile, project.relative(project.profiles_dir))
    else:
        rail = Status("Rail Profile", False, "Not configured", "example configs/rail.ini in use")
    if build.rail_main is None:
        simulator = Status("Simulator", False, "Not built", "cmake --preset release")
    elif project.config_error:
        simulator = Status("Simulator", False, "Config error", project.config_error)
    else:
        simulator = Status("Simulator", True, "Ready", project.relative(build.directory))
    if policy.verified:
        artifact = Status("Policy", True, "Verified", policy.model_id)
    elif not policy.artifacts:
        artifact = Status("Policy", False, "No manifest", project.relative(policy.directory))
    elif policy.missing:
        artifact = Status("Policy", False, "Missing files", ", ".join(a.name for a in policy.missing))
    else:
        artifact = Status("Policy", False, "Hash mismatch", ", ".join(a.name for a in policy.artifacts if not a.ok))
    if build.simulation_only is False:
        controller = Status("Motion Controller", False, "Not probed", "hardware build")
    else:
        controller = Status("Motion Controller", False, "Simulation only", "RVD_SIMULATION_ONLY=ON")
    return [
        rail,
        simulator,
        artifact,
        controller,
        Status("Motion Capture", False, "No driver"),
        Status("LiDAR / Camera", False, "No driver"),
    ]
