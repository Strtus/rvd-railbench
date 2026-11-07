from __future__ import annotations

from rich.console import Group
from rich.text import Text
from textual import on
from textual.app import ComposeResult
from textual.containers import Horizontal, VerticalScroll
from textual.widgets import Button, Footer, Input, Static

from ..config import Vec3, envelope_violations, hill_to_rail, rail_to_hill
from .base import DIM, Page, heading
from .overview import axis_table, mapping_table


class MotionScreen(Page):
    TITLE_TEXT = "Motion Control"

    def compose(self) -> ComposeResult:
        yield from self.compose_title()
        with VerticalScroll(classes="body"):
            yield Static(id="motion-notice", classes="notice")
            yield Static(id="motion-tables")
            yield Static(heading("Frame conversion"), classes="section-title")
            for side in ("hill", "rail"):
                with Horizontal(classes="vec-row"):
                    yield Static("Hill [m]" if side == "hill" else "Rail [m]", classes="vec-label")
                    for axis in "xyz":
                        yield Input(id=f"{side}-{axis}", classes="vec-input", compact=True,
                                    placeholder=axis, restrict=r"[-+0-9.eE]*")
            yield Static(id="envelope", classes="envelope")
            with Horizontal(classes="button-row"):
                yield Button("Use as simulation start", id="use-start", compact=True)
        yield Footer()

    def on_mount(self) -> None:
        self.synced_with: tuple = ()

    def on_screen_resume(self) -> None:
        p = self.project
        stamp = (p.rail_profile, tuple(p.profile.files))
        if stamp != self.synced_with:
            self.synced_with = stamp
            self.show_hill(p.profile.simulation.start_hill_m)
        if p.build.simulation_only is False:
            notice = "Hardware build detected. This page does not command motion."
        else:
            notice = "No motion controller driver in this build (RVD_SIMULATION_ONLY=ON). Motion commands are disabled."
        source = f"rail profile {p.rail_profile}" if p.rail_profile else "the example configuration"
        notice += f"\nLimits and mapping below come from {source}; change them in [6] Rail Setup."
        self.query_one("#motion-notice", Static).update(Text(notice, style=DIM))
        self.query_one("#motion-tables", Static).update(Group(
            heading("Rail envelope"), axis_table(p.profile.rail), Text(""),
            heading("Hill to rail mapping"), mapping_table(p.profile.rail), Text("")))

    def read(self, side: str) -> Vec3 | None:
        try:
            x, y, z = (float(self.query_one(f"#{side}-{a}", Input).value) for a in "xyz")
        except ValueError:
            return None
        return (x, y, z)

    def write(self, side: str, vec: Vec3) -> None:
        for axis, value in zip("xyz", vec):
            self.query_one(f"#{side}-{axis}", Input).value = f"{value:.6g}"

    def show_hill(self, hill: Vec3) -> None:
        self.write("hill", hill)
        rail = hill_to_rail(self.project.profile.rail, hill)
        self.write("rail", rail)
        self.check(rail)

    def check(self, rail: Vec3 | None) -> None:
        if rail is None:
            message = Text("enter three numbers", style=DIM)
        else:
            violations = envelope_violations(self.project.profile.rail, rail)
            message = Text("outside envelope: " + "; ".join(violations), style="bold") if violations \
                else Text("inside rail envelope", style=DIM)
        self.query_one("#envelope", Static).update(message)

    @on(Input.Changed, ".vec-input")
    def changed(self, event: Input.Changed) -> None:
        if not event.input.has_focus:  # programmatic update of the other row
            return
        side = (event.input.id or "").split("-")[0]
        vec = self.read(side)
        if vec is None:
            self.check(None)
            return
        rail_config = self.project.profile.rail
        if side == "hill":
            rail = hill_to_rail(rail_config, vec)
            self.write("rail", rail)
        else:
            rail = vec
            self.write("hill", rail_to_hill(rail_config, vec))
        self.check(rail)

    @on(Button.Pressed, "#use-start")
    def use_start(self) -> None:
        hill = self.read("hill")
        if hill is None:
            self.notify("Hill position is incomplete", severity="warning")
            return
        self.lab.start_override = hill
        self.notify(f"Simulation start set to ({hill[0]:g}, {hill[1]:g}, {hill[2]:g}) m")
