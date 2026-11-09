from __future__ import annotations

from typing import Any

from rich import box
from rich.console import Group
from rich.table import Table
from rich.text import Text
from textual import on
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.screen import ModalScreen
from textual.widgets import Button, Static

from .base import DIM, heading


class ConfirmModal(ModalScreen[bool]):
    BINDINGS = [Binding("escape", "cancel", "Cancel")]

    def __init__(self, title: str, text: str, confirm: str) -> None:
        super().__init__()
        self._title, self._text, self._confirm = title, text, confirm

    def compose(self) -> ComposeResult:
        with Vertical(id="confirm"):
            yield Static(heading(self._title))
            yield Static(Text(self._text))
            with Horizontal(classes="button-row"):
                yield Button(self._confirm, id="confirm-yes", compact=True)
                yield Button("Cancel", id="confirm-no", compact=True)

    def on_mount(self) -> None:
        self.query_one("#confirm-no", Button).focus()

    @on(Button.Pressed, "#confirm-yes")
    def yes(self) -> None:
        self.dismiss(True)

    @on(Button.Pressed, "#confirm-no")
    def action_cancel(self) -> None:
        self.dismiss(False)


def _number(value: Any, digits: int = 4) -> str:
    return "-" if value is None else f"{value:.{digits}f}"


def node_view(state: dict[str, Any], capabilities: dict[str, Any]) -> Group:
    control = state.get("control")
    encoders = state["sensors"]["encoders"]
    facts = Table(box=None, show_header=False, pad_edge=False, padding=(0, 2, 0, 0))
    facts.add_column(style=DIM, no_wrap=True)
    facts.add_column()
    facts.add_row("Node", f"{state['backend']} · " + ("hardware connected" if state["hardware_connected"]
                                                      else "hardware NOT connected")
                  + " · " + ("motion enabled" if state["motion_enabled"] else "read-only"))
    facts.add_row("State", f"{state['state']} ({state['reason']})")
    if state.get("fault"):
        facts.add_row("Fault", Text(state["fault"], style="bold"))
    facts.add_row("Control", "free" if not control else
                  f"{control['owner']} · {control['mode']} · {' '.join(control['axes'])}")
    age = encoders.get("age_s")
    facts.add_row("Encoders", ("ok" if encoders["ok"] else f"error: {encoders['error']}")
                  + (f" · {age * 1e3:.0f} ms old" if age is not None else ""))
    job = state.get("job")
    if job:
        facts.add_row("Task", f"{job['status']} · waypoint {min(job['waypoint'] + 1, job['count'])}/{job['count']}")

    axes = Table(box=box.SIMPLE_HEAD, show_edge=False, pad_edge=False, header_style="bold")
    for name in ("Axis", "Position", "Target", "Limits", "Max speed"):
        axes.add_column(name, justify="left" if name == "Axis" else "right")
    for name, axis in capabilities["axes"].items():
        unit = axis["unit"]
        axes.add_row(name + ("" if axis["motion"] else " (read)"), f"{_number(state['position'].get(name), 6)} {unit}",
                     _number(state["target"].get(name), 6), f"{axis['minimum']:g} … {axis['maximum']:g}",
                     f"{axis['max_speed']:g} {unit}/s")
    parts: list[Any] = [facts, Text(""), axes]

    mocap = state["sensors"].get("mocap")
    if mocap:
        trackers = Table(box=box.SIMPLE_HEAD, show_edge=False, pad_edge=False, header_style="bold")
        for name in ("Tracker", "Status", "Frame", "x", "y", "z"):
            trackers.add_column(name, justify="left" if name in ("Tracker", "Status", "Frame") else "right")
        for name, tracker in mocap["trackers"].items():
            where = tracker.get("rail") or dict(zip("xyz", tracker.get("position_raw") or [None] * 3))
            status = "ok" if tracker["valid"] else tracker.get("reason") or "invalid"
            trackers.add_row(name, status, "rail" if "rail" in tracker else "raw",
                             *(_number(where.get(a)) for a in "xyz"))
        link = "connected" if mocap["connected"] else f"not connected: {mocap['error']}"
        parts += [Text(""), Text(f"Motion capture  {mocap['source']} · {link} · {mocap['frames']} frames", style=DIM),
                  trackers]
    return Group(*parts)
