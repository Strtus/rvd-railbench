from __future__ import annotations

import platform
from importlib.metadata import PackageNotFoundError, version

from rich import box
from rich.console import Group
from rich.table import Table
from rich.text import Text
from textual.app import ComposeResult
from textual.containers import VerticalScroll
from textual.widgets import Footer, Static

from .. import __version__
from ..config import RailConfig
from ..experiments import list_experiments
from ..project import system_status
from .base import DIM, Page, kv_table, section

HILL_AXES = ("x  radial / docking axis", "y  along-track", "z  cross-track")


def axis_table(rail: RailConfig) -> Table:
    table = Table(box=box.SIMPLE_HEAD, show_edge=False, pad_edge=False, header_style="bold")
    for name in ("Rail axis", "Min [m]", "Max [m]", "Travel [m]", "Speed [m/s]", "Accel [m/s²]"):
        table.add_column(name, justify="left" if name == "Rail axis" else "right")
    for i, axis in enumerate("XYZ"):
        table.add_row(axis, f"{rail.minimum_m[i]:g}", f"{rail.maximum_m[i]:g}",
                      f"{rail.maximum_m[i] - rail.minimum_m[i]:g}", f"{rail.velocity_m_s[i]:g}",
                      f"{rail.acceleration_m_s2[i]:g}")
    return table


def mapping_table(rail: RailConfig) -> Table:
    table = Table(box=box.SIMPLE_HEAD, show_edge=False, pad_edge=False, header_style="bold")
    for name in ("Hill axis", "Rail axis", "Sign", "Scale [rail m / Hill m]", "Port on rail [m]"):
        table.add_column(name, justify="left" if name == "Hill axis" else "right")
    for j in range(3):
        a = rail.axis[j]
        table.add_row(HILL_AXES[j], "XYZ"[a], f"{rail.sign[j]:+g}", f"{rail.scale[j]:g}", f"{rail.target_m[a]:g}")
    return table


def _package(name: str) -> str:
    try:
        return version(name)
    except PackageNotFoundError:
        return "?"


class OverviewScreen(Page):
    TITLE_TEXT = "System Overview"

    def compose(self) -> ComposeResult:
        yield from self.compose_title()
        with VerticalScroll(classes="body"):
            yield Static(id="overview")
        yield Footer()

    def on_screen_resume(self) -> None:
        self.render_view()

    def render_view(self) -> None:
        p = self.project
        build, git, policy, profile = p.build, p.git, p.policy, p.profile
        dirty = "" if git.dirty is None else (" (uncommitted changes)" if git.dirty else " (clean)")
        software = kv_table([
            ("Project", f"RVD-RailBench {p.version}", p.root.as_posix()),
            ("Source", f"{git.branch} @ {git.commit}{dirty}" if git.commit else "not a git checkout", ""),
            ("Build", p.relative(build.directory) or "not built",
             f"{build.build_type}, RVD_SIMULATION_ONLY={build.cache.get('RVD_SIMULATION_ONLY', '?')}"
             if build.directory else "cmake --preset release && cmake --build --preset release"),
            ("Executables", ", ".join(n for n, f in (("rail_main", build.rail_main), ("rail_replay", build.rail_replay)) if f) or "none", ""),
            ("Policy backends", "builtin" + (", onnx" if build.onnx else ""), f"configured: {profile.controller.backend}"),
            ("Interface", f"rvdlab {__version__}", f"textual {_package('textual')}, python {platform.python_version()}"),
        ])

        manifest = policy.manifest
        actuator = dict(manifest.get("actuator_profile", {}))
        gains = actuator.get("scale")
        if isinstance(gains, list) and gains and all(g == gains[0] for g in gains):
            actuator["scale"] = gains[0]
        policy_rows = [
            ("Model", policy.model_id, p.relative(policy.directory)),
            ("Controller", f"{manifest.get('controller', '?')}, period {manifest.get('period_s', '?')} s",
             f"obs {manifest.get('observation_dimension', '?')} → action {manifest.get('action_dimension', '?')}, "
             f"history {manifest.get('history_length', '?')}"),
            ("Virtual thruster", f"lag {actuator.get('tau', '?')} s, gain {actuator.get('scale', '?')}",
             f"deadzone {actuator.get('deadzone', '?')} N"),
        ]
        for artifact in policy.artifacts:
            state = "sha256 verified" if artifact.ok else ("missing" if artifact.actual is None else "sha256 MISMATCH")
            policy_rows.append((artifact.name, state, artifact.expected[:16] + "…"))

        rail = profile.rail
        files = Text("\n".join(p.relative(f) for f in profile.files) or "built-in defaults", style=DIM)
        timing = kv_table([
            ("Measurement age", f"{rail.measurement_age_s * 1e3:g} ms", "stale above"),
            ("Measurement late", f"{rail.measurement_late_s * 1e3:g} ms", "late above"),
            ("Clock skew", f"{rail.clock_skew_s * 1e3:g} ms", ""),
            ("Communication timeout", f"{rail.communication_timeout_s * 1e3:g} ms", ""),
            ("Tracking error", f"{rail.tracking_error_m * 1e3:g} mm", "stop above"),
            ("Observation warning", f"{profile.controller.ood_warn_z:g} σ", ""),
        ])
        configuration: list = [files, Text("")]
        if p.config_error:
            configuration += [Text(f"configuration error: {p.config_error}", style="bold"), Text("")]
        configuration += [axis_table(rail), Text(""), mapping_table(rail), Text(""), timing]

        devices = kv_table([(s.label, ("● " if s.ok else "○ ") + s.text, s.detail) for s in system_status(p)])
        hardware = Group(devices, Text(""), Text(
            "No physical device driver exists in this build; rail_main rejects --lab before creating any device.",
            style=DIM))

        records = list_experiments(p.experiments_dir)
        latest = records[0] if records else None
        experiments = kv_table([
            ("Directory", p.relative(p.experiments_dir), ""),
            ("Recorded", str(len(records)), ""),
            ("Latest", latest.id if latest else "none", f"{latest.outcome} {latest.reason}" if latest else ""),
        ])

        self.query_one("#overview", Static).update(Group(
            section("Software", software),
            section("Policy", kv_table(policy_rows)),
            section("Configuration", Group(*configuration)),
            section("Devices", hardware),
            section("Experiments", experiments),
        ))
