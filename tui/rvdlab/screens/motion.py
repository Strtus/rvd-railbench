from __future__ import annotations

import threading
from typing import Any

from rich.console import Group
from rich.text import Text
from textual import on, work
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, VerticalScroll
from textual.widgets import Button, Footer, Input, Static

from ..config import Vec3, envelope_violations, hill_to_rail, rail_to_hill
from ..remote import AXES, connect, move
from .base import DIM, Page, heading
from .node_view import ConfirmModal, node_view
from .overview import axis_table, mapping_table

NUMBER = r"[-+0-9.eE]*"


class MotionScreen(Page):
    TITLE_TEXT = "Motion Control"
    BINDINGS = [*Page.BINDINGS, Binding("f9", "node_stop", "Stop rail")]

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
            yield Static(heading("Rail node"), classes="section-title")
            yield Static(id="node-view")
            with Horizontal(classes="vec-row"):
                yield Static("Move by [m]", classes="vec-label")
                for axis in AXES:
                    yield Input(id=f"by-{axis}", classes="vec-input", compact=True, placeholder=axis, restrict=NUMBER)
            with Horizontal(classes="vec-row"):
                yield Static("Speed [m/s]", classes="vec-label")
                yield Input(id="move-speed", classes="vec-input", compact=True, placeholder="0.002", restrict=NUMBER)
            with Horizontal(classes="button-row"):
                yield Button("Move by", id="node-move-by", compact=True)
                yield Button("Move to Rail [m]", id="node-move-to", compact=True)
                yield Button("Stop rail  F9", id="node-stop", compact=True)
            yield Static(id="node-message", classes="context")
        yield Footer()

    def on_mount(self) -> None:
        self.synced_with: tuple = ()
        self.node_path = self.lab.node_path
        self.node: Any = None
        self.capabilities: dict[str, Any] | None = None
        self.state: dict[str, Any] | None = None
        self.cancel: threading.Event | None = None
        self.visible = False
        if self.node_path is None:
            self.query_one("#node-view", Static).update(Text(
                "No rail node connection. Put the access folder from the node administrator in ~/.rvdlab/node/ "
                "or start rvdlab --node PATH/connection.json.", style=DIM))
        else:
            try:
                self.node = connect(self.node_path)
            except Exception as error:
                self.query_one("#node-view", Static).update(Text(f"{self.node_path}: {error}", style="bold"))
            self.set_interval(0.5, self.poll_node)

    def on_screen_suspend(self) -> None:
        self.visible = False

    def on_screen_resume(self) -> None:
        self.visible = True
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

    # rail node

    @work(thread=True, exclusive=True, group="node-poll")
    def poll_node(self) -> None:
        if self.node is None or not self.visible:
            return
        try:
            capabilities = self.capabilities or self.node.capabilities()
            state = self.node.state()
        except Exception as error:
            self.capabilities = None
            self.app.call_from_thread(self.show_node, None, str(error))
            return
        self.capabilities, self.state = capabilities, state
        self.app.call_from_thread(self.show_node, state, None)

    def show_node(self, state: dict[str, Any] | None, error: str | None) -> None:
        view = self.query_one("#node-view", Static)
        if state is None or self.capabilities is None:
            view.update(Text(f"{self.node_path}: {error}", style="bold"))
        else:
            view.update(node_view(state, self.capabilities))

    def say(self, message: str) -> None:
        self.query_one("#node-message", Static).update(Text(message, style=DIM))

    def speed(self) -> float | None:
        try:
            value = float(self.query_one("#move-speed", Input).value)
        except ValueError:
            return None
        return value if value > 0 else None

    def request_move(self, goal: dict[str, float], what: str) -> None:
        if self.node is None or self.state is None:
            self.notify("The rail node is not connected", severity="warning")
            return
        speed = self.speed()
        if speed is None:
            self.notify("Enter a positive speed", severity="warning")
            return
        here = [self.state["position"].get(a) for a in AXES]
        rail = tuple(goal.get(a, h if h is not None else 0.0) for a, h in zip(AXES, here))
        violations = envelope_violations(self.project.profile.rail, rail)  # type: ignore[arg-type]
        if violations:
            self.notify("outside the rail envelope: " + "; ".join(violations), severity="error")
            return
        lines = [f"{a}: {self.state['position'].get(a):.6f} -> {v:.6f} m" for a, v in goal.items()]
        text = (f"{what} at {speed:g} m/s on the real rail.\n" + "\n".join(lines)
                + "\n\nWatch the rail and keep the emergency stop within reach. F9 stops it.")
        self.app.push_screen(ConfirmModal("Move the rail", text, "Move"),
                             lambda ok: self.run_move(goal, speed) if ok else None)

    @on(Button.Pressed, "#node-move-by")
    def move_by(self) -> None:
        if self.state is None:
            self.notify("The rail node is not connected", severity="warning")
            return
        goal = {}
        for axis in AXES:
            text = self.query_one(f"#by-{axis}", Input).value.strip()
            if text:
                try:
                    delta = float(text)
                except ValueError:
                    self.notify(f"{axis}: not a number", severity="warning")
                    return
                if delta:
                    goal[axis] = self.state["position"][axis] + delta
        if not goal:
            self.notify("Enter an offset for at least one axis", severity="warning")
            return
        self.request_move(goal, "Move by the given offsets")

    @on(Button.Pressed, "#node-move-to")
    def move_to(self) -> None:
        rail = self.read("rail")
        if rail is None:
            self.notify("Rail position is incomplete", severity="warning")
            return
        self.request_move(dict(zip(AXES, rail)), "Move to the Rail [m] position")

    @work(thread=True, exclusive=True, group="node-move")
    def run_move(self, goal: dict[str, float], speed: float) -> None:
        self.cancel = threading.Event()
        self.app.call_from_thread(self.say, "moving ...")
        try:
            status = move(connect(self.node_path), goal, speed, self.cancel)  # type: ignore[arg-type]
        except Exception as error:
            status = f"failed: {error}"
        self.app.call_from_thread(self.say, f"move {status}")

    @on(Button.Pressed, "#node-stop")
    def action_node_stop(self) -> None:
        if self.cancel is not None:
            self.cancel.set()
        if self.node_path is not None:
            self.stop_node()

    @work(thread=True, group="node-stop")
    def stop_node(self) -> None:
        try:
            state = connect(self.node_path).stop()  # type: ignore[arg-type]
            message = f"stopped · {state['state']}" + (f" · fault {state['fault']}" if state["fault"] else "")
        except Exception as error:
            message = f"STOP FAILED: {error}. Use the emergency stop."
        self.app.call_from_thread(self.say, message)
