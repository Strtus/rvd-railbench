from __future__ import annotations

from typing import TYPE_CHECKING

from rich.console import Group, RenderableType
from rich.table import Table
from rich.text import Text
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal
from textual.screen import Screen
from textual.widgets import Static

if TYPE_CHECKING:
    from ..app import RvdLabApp
    from ..project import Project

DIM = "#8a8a8a"


def heading(title: str) -> Text:
    return Text(title.upper(), style="bold #e6e6e6")


def kv_table(rows: list[tuple[str, str, str]]) -> Table:
    table = Table(box=None, show_header=False, pad_edge=False, padding=(0, 2, 0, 0))
    table.add_column(style=DIM, no_wrap=True)
    table.add_column(no_wrap=True)
    table.add_column(style=DIM)
    for row in rows:
        table.add_row(*row)
    return table


def section(title: str, body: RenderableType) -> Group:
    return Group(heading(title), body, Text(""))


class Page(Screen):
    TITLE_TEXT = ""
    BINDINGS = [Binding("escape", "app.pop_screen", "Back")]

    @property
    def lab(self) -> RvdLabApp:
        return self.app  # type: ignore[return-value]

    @property
    def project(self) -> Project:
        return self.lab.project

    def compose_title(self) -> ComposeResult:
        mode = "simulation" if self.project.build.simulation_only is not False else "hardware build"
        with Horizontal(classes="title-bar"):
            yield Static(f"RVDLAB  ›  {self.TITLE_TEXT}", classes="title-left")
            yield Static(f"RVD-RailBench v{self.project.version} · {mode}", classes="title-right")
