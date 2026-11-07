from __future__ import annotations

from rich.text import Text
from textual import events, on
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.message import Message
from textual.reactive import reactive
from textual.screen import Screen
from textual.widget import Widget
from textual.widgets import Input, Static

from ..logo import LOGO, SUBTITLE, WIDTH
from ..project import Status, system_status

MENU = (
    ("1", "System Overview", "overview"),
    ("2", "Motion Control", "motion"),
    ("3", "Autonomous Rendezvous", "rendezvous"),
    ("4", "Sensor & Telemetry", "sensors"),
    ("5", "Experiment Manager", "manager"),
    ("6", "Rail Setup", "setup"),
)


def status_text(statuses: list[Status]) -> Text:
    text = Text(no_wrap=True, overflow="crop")
    for i, status in enumerate(statuses):
        text.append(f"  {status.label:<22}")
        text.append("● " if status.ok else "○ ", style="bold #ffffff" if status.ok else "#6a6a6a")
        text.append(status.text, style="#e6e6e6" if status.ok else "#8a8a8a")
        if i < len(statuses) - 1:
            text.append("\n")
    return text


class Menu(Widget):
    selected: reactive[int] = reactive(0)

    class Chosen(Message):
        def __init__(self, index: int) -> None:
            super().__init__()
            self.index = index

    def render(self) -> Text:
        text = Text(no_wrap=True, overflow="crop")
        for i, (key, label, _) in enumerate(MENU):
            if i == self.selected:
                text.append("› ", style="bold #ffffff")
                text.append(f"[{key}] {label}", style="bold #ffffff")
            else:
                text.append(f"  [{key}] {label}", style="#bdbdbd")
            if i < len(MENU) - 1:
                text.append("\n")
        return text

    def on_click(self, event: events.Click) -> None:
        if 0 <= event.y < len(MENU):
            self.selected = event.y
            self.post_message(self.Chosen(event.y))


class HomeScreen(Screen):
    BINDINGS = [
        Binding("up", "move(-1)", show=False, priority=True),
        Binding("down", "move(1)", show=False, priority=True),
    ]

    def compose(self) -> ComposeResult:
        project = self.app.project  # type: ignore[attr-defined]
        with Vertical(id="splash"):
            yield Static(Text(LOGO, style="bold #ededed", no_wrap=True), id="logo")
            yield Static(SUBTITLE.center(WIDTH).rstrip(), id="subtitle")
            yield Static(f"RVD-RailBench v{project.version}".center(WIDTH).rstrip(), id="version")
            yield Static(status_text(system_status(project)), id="status")
            yield Menu(id="menu")
            with Horizontal(id="prompt-row"):
                yield Static(">", id="prompt")
                yield Input(id="command", compact=True, select_on_focus=False)
            yield Static("", id="message")

    def on_mount(self) -> None:
        self._shown = False
        self.query_one("#command", Input).focus()

    def on_screen_resume(self) -> None:
        if not self._shown:
            self._shown = True
            return
        self.app.reload_project()  # type: ignore[attr-defined]
        project = self.app.project  # type: ignore[attr-defined]
        self.query_one("#status", Static).update(status_text(system_status(project)))
        self.query_one("#command", Input).focus()

    def action_move(self, step: int) -> None:
        menu = self.query_one(Menu)
        menu.selected = (menu.selected + step) % len(MENU)

    @on(Input.Submitted, "#command")
    def submitted(self, event: Input.Submitted) -> None:
        command = event.value.strip().lower()
        event.input.value = ""
        if not command:
            self.open(self.query_one(Menu).selected)
            return
        if command in ("q", "quit", "exit"):
            self.app.exit()
            return
        for i, (key, label, _) in enumerate(MENU):
            if command in (key, label.lower()):
                self.open(i)
                return
        self.query_one("#message", Static).update(Text(f"  unknown command: {command}   (1-6, q)", style="#7a7a7a"))

    @on(Menu.Chosen)
    def chosen(self, event: Menu.Chosen) -> None:
        self.open(event.index)

    def open(self, index: int) -> None:
        self.query_one(Menu).selected = index
        self.query_one("#message", Static).update("")
        self.app.push_screen(MENU[index][2])
