from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path

from rich import box
from rich.console import Group, RenderableType
from rich.table import Table
from rich.text import Text
from textual import on, work
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, VerticalScroll
from textual.widgets import Button, Footer, Input, Select, Static

from ..config import ConfigError, Profile, RailConfig, Vec3, apply_file, envelope_violations, hill_to_rail
from ..profiles import RailProfile, check, list_profiles, load_rail_profile, profile_path, save, set_active
from ..project import ProjectError
from ..sweep import CANONICAL_STARTS, REFERENCE, Check, Fit, evaluate, fit_mapping, run_sweep, validation_starts
from .base import DIM, Page, heading

NUMBER = r"[-+0-9.eE]*"
EXAMPLE = "__example__"
HILL_LABELS = ("x  radial · docking axis", "y  along-track", "z  cross-track")
KEY = 26


def fmt(value: float) -> str:
    text = repr(float(value))
    return text[:-2] if text.endswith(".0") else text


def _notes(path: Path) -> tuple[str, ...]:
    lines = path.read_text(encoding="utf-8").splitlines()
    notes = []
    for line in lines[1:]:
        if not line.startswith("#"):
            break
        notes.append(line.lstrip("# ").rstrip())
    return tuple(notes)


class SetupScreen(Page):
    TITLE_TEXT = "Rail Setup"
    BINDINGS = [*Page.BINDINGS, Binding("ctrl+s", "save", "Save")]

    def compose(self) -> ComposeResult:
        yield from self.compose_title()
        with VerticalScroll(classes="body"):
            yield Static(id="setup-intro", classes="notice")
            with Horizontal(classes="setup-row"):
                yield Static("profile name", classes="setup-key")
                yield Input(id="profile-name", compact=True, restrict=r"[A-Za-z0-9_.-]*", max_length=64)
                yield Static("load", classes="setup-load-key")
                yield Select([], id="profile-load", compact=True, prompt="example or saved profile")
            yield Static(heading("Rail axes"), classes="section-title")
            yield Static(" " * KEY + "".join(h.ljust(15) for h in ("min [m]", "max [m]", "speed [m/s]", "accel [m/s²]")),
                         classes="setup-header")
            for axis in "XYZ":
                with Horizontal(classes="setup-row"):
                    yield Static(axis, classes="setup-key")
                    for field in ("min", "max", "speed", "accel"):
                        yield Input(id=f"{field}-{axis.lower()}", classes="cell", compact=True, restrict=NUMBER)
            yield Static(heading("Hill mapping"), classes="section-title")
            yield Static(" " * KEY + "rail axis".ljust(10) + "sign".ljust(9) + "scale [rail m / Hill m]".ljust(25)
                         + "port on rail [m]", classes="setup-header")
            for j, label in enumerate(HILL_LABELS):
                with Horizontal(classes="setup-row"):
                    yield Static(label, classes="setup-key")
                    yield Select([("X", "0"), ("Y", "1"), ("Z", "2")], id=f"axis-{j}", allow_blank=False,
                                 compact=True, classes="axis-select")
                    yield Select([("+1", "1"), ("-1", "-1")], id=f"sign-{j}", allow_blank=False, compact=True,
                                 classes="sign-select")
                    yield Input(id=f"scale-{j}", classes="cell wide", compact=True, restrict=NUMBER)
                    yield Input(id=f"port-{j}", classes="cell", compact=True, restrict=NUMBER)
            yield Static(heading("Simulation start"), classes="section-title")
            with Horizontal(classes="setup-row"):
                yield Static("Hill frame [m]", classes="setup-key")
                for axis in "xyz":
                    yield Static(axis, classes="setup-axis")
                    yield Input(id=f"start-{axis}", classes="cell short", compact=True, restrict=NUMBER)
                yield Static("margin [%]", classes="setup-axis wide")
                yield Input("10", id="margin", classes="cell short", compact=True, restrict=r"[0-9.]*")
            with Horizontal(classes="button-row"):
                yield Button("Suggest mapping", id="suggest", compact=True)
                yield Button("Validate", id="validate", compact=True)
                yield Button("Save and use", id="save", compact=True)
            yield Static(id="setup-result", classes="setup-result")
        yield Footer()

    def on_mount(self) -> None:
        self.loaded_name: str | None = None
        self.base_rail = RailConfig()
        self.notes: tuple[str, ...] = ()
        self.busy = False
        self.refresh_choices()
        if self.project.rail_profile:
            self.load_named(self.project.rail_profile)
        else:
            self.load_example()

    def on_screen_resume(self) -> None:
        p = self.project
        if p.explicit_configs:
            intro = "This session was started with --config, so rail profiles are not used until it is restarted without it."
        elif p.rail_profile:
            intro = (f"Active rail profile: {p.rail_profile} ({p.relative(profile_path(p.profiles_dir, p.rail_profile))}). "
                     "Timing and tracking thresholds are kept from the loaded profile.")
        else:
            intro = ("No rail profile yet. Enter your rail's travel, limits and mapping, starting from the example. "
                     "Suggest mapping fits the Hill mapping to your rail; Validate runs the deployment start set "
                     "in simulation; Save and use makes it the active profile.")
        self.query_one("#setup-intro", Static).update(Text(intro, style=DIM))

    # form <-> profile

    def refresh_choices(self) -> None:
        options = [("example · configs/rail.ini", EXAMPLE)]
        options += [(name, name) for name in list_profiles(self.project.profiles_dir)]
        self.query_one("#profile-load", Select).set_options(options)

    @on(Select.Changed, "#profile-load")
    def load_choice(self, event: Select.Changed) -> None:
        if event.value == EXAMPLE:
            self.load_example()
        elif isinstance(event.value, str):
            self.load_named(event.value)

    def load_example(self) -> None:
        profile = Profile()
        try:
            for name in ("rail.ini", "simulation.ini"):
                apply_file(profile, self.project.root / "configs" / name)
        except (ConfigError, OSError) as error:
            self.show(Text(f"cannot read the example: {error}", style="bold"))
        self.loaded_name, self.base_rail, self.notes = None, profile.rail, ()
        self.fill(RailProfile("my-rail", profile.rail, profile.simulation.start_hill_m))

    def load_named(self, name: str) -> None:
        path = profile_path(self.project.profiles_dir, name)
        try:
            profile = load_rail_profile(path)
        except (ConfigError, OSError) as error:
            self.show(Text(f"cannot read {path}: {error}", style="bold"))
            return
        self.loaded_name, self.base_rail, self.notes = name, profile.rail, _notes(path)
        self.fill(profile)

    def fill(self, profile: RailProfile) -> None:
        r = profile.rail
        self.query_one("#profile-name", Input).value = profile.name
        for i, axis in enumerate("xyz"):
            for field, values in (("min", r.minimum_m), ("max", r.maximum_m), ("speed", r.velocity_m_s),
                                  ("accel", r.acceleration_m_s2)):
                self.query_one(f"#{field}-{axis}", Input).value = fmt(values[i])
        self.fill_mapping(r)
        self.set_start(profile.start_hill_m)

    def fill_mapping(self, r: RailConfig) -> None:
        for j in range(3):
            self.query_one(f"#axis-{j}", Select).value = str(r.axis[j])
            self.query_one(f"#sign-{j}", Select).value = "1" if r.sign[j] > 0 else "-1"
            self.query_one(f"#scale-{j}", Input).value = fmt(r.scale[j])
            self.query_one(f"#port-{j}", Input).value = fmt(r.target_m[r.axis[j]])

    def set_start(self, start: Vec3) -> None:
        for axis, value in zip("xyz", start):
            self.query_one(f"#start-{axis}", Input).value = fmt(value)

    def number(self, widget_id: str, label: str) -> float:
        text = self.query_one(f"#{widget_id}", Input).value.strip()
        try:
            return float(text)
        except ValueError:
            raise ConfigError(f"{label}: enter a number") from None

    def read_limits(self) -> RailConfig:
        rows = {field: tuple(self.number(f"{field}-{a}", f"{a.upper()} {field}") for a in "xyz")
                for field in ("min", "max", "speed", "accel")}
        axis = tuple(int(str(self.query_one(f"#axis-{j}", Select).value)) for j in range(3))
        sign = tuple(float(str(self.query_one(f"#sign-{j}", Select).value)) for j in range(3))
        if sorted(axis) != [0, 1, 2]:
            raise ConfigError("each Hill axis needs its own rail axis")
        for i, a in enumerate("XYZ"):
            if not rows["min"][i] < rows["max"][i]:
                raise ConfigError(f"{a}: min must be below max")
            if not (rows["speed"][i] > 0 and rows["accel"][i] > 0):
                raise ConfigError(f"{a}: speed and acceleration limits must be positive")
        return replace(self.base_rail, minimum_m=rows["min"], maximum_m=rows["max"], velocity_m_s=rows["speed"],
                       acceleration_m_s2=rows["accel"], axis=axis, sign=sign)  # type: ignore[arg-type]

    def read_form(self) -> RailProfile:
        rail = self.read_limits()
        scale = tuple(self.number(f"scale-{j}", f"{'xyz'[j]} scale") for j in range(3))
        ports = [self.number(f"port-{j}", f"{'xyz'[j]} port") for j in range(3)]
        target = [0.0, 0.0, 0.0]
        for j in range(3):
            target[rail.axis[j]] = ports[j]
        rail = replace(rail, scale=scale, target_m=(target[0], target[1], target[2]))  # type: ignore[arg-type]
        start = (self.number("start-x", "start x"), self.number("start-y", "start y"), self.number("start-z", "start z"))
        profile = RailProfile(self.query_one("#profile-name", Input).value.strip(), rail, start)
        check(profile)
        violations = envelope_violations(rail, hill_to_rail(rail, start))
        if violations:
            raise ConfigError("simulation start maps outside the envelope: " + "; ".join(violations))
        return profile

    # actions

    def show(self, renderable: RenderableType) -> None:
        self.query_one("#setup-result", Static).update(renderable)

    def set_busy(self, busy: bool) -> None:
        self.busy = busy
        for button in self.query(Button):
            button.disabled = busy

    @on(Button.Pressed, "#suggest")
    def _suggest(self) -> None:
        if not self.busy:
            self.suggest()

    @on(Button.Pressed, "#validate")
    def _validate(self) -> None:
        if not self.busy:
            self.validate()

    @on(Button.Pressed, "#save")
    def _save(self) -> None:
        self.action_save()

    def progress(self, stage: str) -> Callable[[int, int], None]:
        return lambda done, total: self.show(Text(f"{stage} {done}/{total} …", style=DIM))

    @work(exclusive=True, group="setup")
    async def suggest(self) -> None:
        try:
            rail = self.read_limits()
            margin = self.number("margin", "margin") / 100
            if not 0 <= margin < 0.9:
                raise ConfigError("margin must be between 0 and 90 %")
        except ConfigError as error:
            self.show(Text(str(error), style="bold"))
            return
        self.set_busy(True)
        try:
            reference = await run_sweep(self.project, RailProfile("reference", REFERENCE, CANONICAL_STARTS[0]),
                                        CANONICAL_STARTS, self.progress("reference trajectories"))
            failed = [r for r in reference if r.outcome != "DOCKED"]
            if failed:
                self.show(Text(f"{len(failed)} reference runs did not dock ({failed[0].outcome} {failed[0].reason}); "
                               "no mapping suggested.", style="bold"))
                return
            fit = fit_mapping(rail, reference, margin)
            fitted = replace(rail, scale=fit.scale, target_m=fit.target)
            self.fill_mapping(fitted)
            self.set_start(CANONICAL_STARTS[4])
            checks = await self.run_checks(fitted)
            self.show(Group(self.fit_table(fit, fitted, margin), Text(""), self.check_report(fitted, checks)))
        except ProjectError as error:
            self.show(Text(str(error), style="bold"))
        finally:
            self.set_busy(False)

    @work(exclusive=True, group="setup")
    async def validate(self) -> None:
        try:
            profile = self.read_form()
        except ConfigError as error:
            self.show(Text(str(error), style="bold"))
            return
        self.set_busy(True)
        try:
            self.show(self.check_report(profile.rail, await self.run_checks(profile.rail)))
        except ProjectError as error:
            self.show(Text(str(error), style="bold"))
        finally:
            self.set_busy(False)

    async def run_checks(self, rail: RailConfig) -> list[Check]:
        starts = validation_starts(rail)
        if not starts:
            return []
        results = await run_sweep(self.project, RailProfile("check", rail, starts[0]), starts,
                                  self.progress("validation runs"))
        return [evaluate(rail, r) for r in results]

    def action_save(self) -> None:
        if self.busy:
            return
        try:
            profile = self.read_form()
        except ConfigError as error:
            self.show(Text(str(error), style="bold"))
            return
        directory = self.project.profiles_dir
        if profile_path(directory, profile.name).exists() and profile.name != self.loaded_name:
            self.show(Text(f"a profile named {profile.name} already exists; load it to replace it or pick another name",
                           style="bold"))
            return
        save(directory, profile, self.notes)
        set_active(directory, profile.name)
        self.loaded_name = profile.name
        self.lab.reload_project()
        self.refresh_choices()
        self.notify(f"Rail profile {profile.name} saved and active")
        self.app.pop_screen()

    # reports

    def fit_table(self, fit: Fit, rail: RailConfig, margin: float) -> Group:
        table = Table(box=box.SIMPLE_HEAD, show_edge=False, pad_edge=False, header_style="bold")
        for name in ("Hill axis", "Rail axis", "Extent [Hill m]", "Peak [Hill m/s]", "Scale", "Limited by", "Port [m]"):
            table.add_column(name, justify="left" if name in ("Hill axis", "Limited by") else "right")
        for j in range(3):
            a = rail.axis[j]
            lo, hi = fit.extent[j]
            table.add_row("xyz"[j], "XYZ"[a], f"{lo:.2f} … {hi:.2f}", f"{fit.peak[j]:.3f}", fmt(fit.scale[j]),
                          fit.limited_by[j], fmt(fit.target[a]))
        return Group(heading("Suggested mapping"),
                     Text(f"Largest scale per axis that keeps the 9 reference approaches inside {100 - margin * 100:.0f}% "
                          "of the travel and of the speed limit.", style=DIM), table)

    def check_report(self, rail: RailConfig, checks: list[Check]) -> Group:
        if not checks:
            return Group(heading("Validation"), Text("No start of the deployment set fits this rail; check the docking-axis "
                                                     "mapping.", style="bold"))
        def mm(value: float) -> str:
            return f"{value * 1e3:.0f}" if math.isfinite(value) else "–"

        docked = sum(1 for c in checks if c.outcome == "DOCKED")
        worst = [max(c.speed_use[i] for c in checks) for i in range(3)]
        verdict = "PASS" if docked == len(checks) else "FAIL"
        summary = Text(f"{verdict} · {docked}/{len(checks)} docked · peak speed use "
                       f"X {worst[0]:.0%}  Y {worst[1]:.0%}  Z {worst[2]:.0%} · minimum clearance "
                       f"{mm(min(c.clearance_m for c in checks))} mm", style="bold" if verdict == "FAIL" else "")
        table = Table(box=box.SIMPLE_HEAD, show_edge=False, pad_edge=False, header_style="bold")
        for name in ("Start x, y, z [Hill m]", "Result", "Steps", "Speed use X / Y / Z", "Clearance [mm]"):
            table.add_column(name, justify="right" if name in ("Steps", "Clearance [mm]") else "left")
        for c in checks:
            table.add_row(", ".join(f"{v:g}" for v in c.start), f"{c.outcome} {c.reason}".strip(), str(c.steps),
                          " / ".join(f"{u:.0%}" for u in c.speed_use), mm(c.clearance_m))
        return Group(heading("Validation"), summary, table)
