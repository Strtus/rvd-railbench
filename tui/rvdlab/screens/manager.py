from __future__ import annotations

import asyncio
import os
import subprocess
import sys

from rich.text import Text
from textual import on, work
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Vertical, VerticalScroll
from textual.screen import ModalScreen
from textual.widgets import DataTable, Footer, Static

from ..experiments import ExperimentRecord, list_experiments
from .base import DIM, Page, heading

COLUMNS = ("Experiment", "Started (UTC)", "Outcome", "Reason", "Steps", "Interventions", "Mode", "Clock", "Checksums")


class TextModal(ModalScreen):
    BINDINGS = [Binding("escape", "app.pop_screen", "Close")]

    def __init__(self, title: str, text: str) -> None:
        super().__init__()
        self._title = title
        self._text = text

    def compose(self) -> ComposeResult:
        with Vertical(id="modal"):
            yield Static(heading(self._title))
            with VerticalScroll():
                yield Static(Text(self._text, no_wrap=True))
            yield Static(Text("Esc to close", style=DIM))


class ManagerScreen(Page):
    TITLE_TEXT = "Experiment Manager"
    BINDINGS = [
        *Page.BINDINGS,
        Binding("v", "verify", "Verify replay"),
        Binding("c", "configuration", "Configuration"),
        Binding("o", "folder", "Open folder"),
        Binding("r", "reload", "Refresh"),
    ]

    def compose(self) -> ComposeResult:
        yield from self.compose_title()
        yield Static(id="manager-context", classes="context")
        yield DataTable(id="experiments", cursor_type="row", show_row_labels=False)
        yield Static(id="manager-status", classes="context")
        yield Footer()

    def on_mount(self) -> None:
        self.records: list[ExperimentRecord] = []
        self.query_one(DataTable).add_columns(*COLUMNS)

    def on_screen_resume(self) -> None:
        self.action_reload()
        self.query_one(DataTable).focus()

    def status(self, message: str) -> None:
        self.query_one("#manager-status", Static).update(Text(message, style=DIM))

    def action_reload(self) -> None:
        p = self.project
        self.records = list_experiments(p.experiments_dir)
        table = self.query_one(DataTable)
        table.clear()
        for r in self.records:
            clock = "virtual" if r.clock.startswith("virtual") else ("real time" if r.clock else "–")
            table.add_row(
                r.id, r.started.replace("T", " ")[:19], r.outcome, r.reason or "–",
                "–" if r.steps is None else str(r.steps),
                "–" if r.interventions is None else str(r.interventions),
                r.mode or "–", clock, "ok" if r.complete else "–", key=r.id)
        self.query_one("#manager-context", Static).update(Text(
            f"{p.relative(p.experiments_dir)} · {len(self.records)} experiments · Enter opens telemetry", style=DIM))
        self.status("" if self.records else "No experiments yet. Run one from Autonomous Rendezvous.")

    def selected(self) -> ExperimentRecord | None:
        row = self.query_one(DataTable).cursor_row
        return self.records[row] if 0 <= row < len(self.records) else None

    @on(DataTable.RowSelected)
    def open_telemetry(self) -> None:
        record = self.selected()
        if record is not None:
            self.app.get_screen("sensors").show(record.path)  # type: ignore[attr-defined]
            self.app.push_screen("sensors")

    def action_configuration(self) -> None:
        record = self.selected()
        if record is None:
            return
        path = record.path / "configuration.ini"
        text = path.read_text(encoding="utf-8") if path.is_file() else "configuration.ini is missing"
        self.app.push_screen(TextModal(f"{record.id} · configuration", text))

    def action_folder(self) -> None:
        record = self.selected()
        if record is None:
            return
        if os.name == "nt":
            os.startfile(record.path)  # type: ignore[attr-defined]
        else:
            subprocess.Popen(["open" if sys.platform == "darwin" else "xdg-open", str(record.path)])

    def action_verify(self) -> None:
        record = self.selected()
        if record is None:
            return
        if self.project.build.rail_replay is None:
            self.status("rail_replay is not built")
            return
        self.replay(record)

    @work(exclusive=True, group="replay")
    async def replay(self, record: ExperimentRecord) -> None:
        self.status(f"{record.id}: replaying recorded controller inputs …")
        process = await asyncio.create_subprocess_exec(
            str(self.project.build.rail_replay), str(record.path), str(self.project.policy.directory),
            cwd=str(self.project.root), stdin=asyncio.subprocess.DEVNULL,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        output, _ = await process.communicate()
        verdict = "PASS" if process.returncode == 0 else "FAIL"
        self.status(f"{record.id}: replay {verdict} · {output.decode('utf-8', 'replace').strip()}")
