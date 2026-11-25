from __future__ import annotations

import csv
from pathlib import Path

from rich.console import Group
from rich.text import Text
from textual import on
from textual.app import ComposeResult
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.widgets import Button, Footer, Input, Sparkline, Static

from ..config import EXTERNAL_CONTROLLER, read_snapshot
from ..experiments import list_experiments
from ..plot import VelocitySparkline
from ..telemetry import EncoderSpeed, Series, load, num, resolve_csv, summarize
from .base import DIM, Page, kv_table, section

SPARKS = (
    ("distance", "distance [m]"),
    ("speed", "Hill velocity XYZ [m/s]"),
    ("track", "tracking error [mm]"),
    ("age", "measurement age [ms]"),
    ("sigma", "observation σ"),
)


class SensorsScreen(Page):
    TITLE_TEXT = "Sensor & Telemetry"

    def __init__(self) -> None:
        super().__init__()
        self.path: Path | None = None

    def compose(self) -> ComposeResult:
        yield from self.compose_title()
        with Horizontal(id="source-row"):
            yield Static("source", classes="form-key wide")
            yield Input(id="source", compact=True,
                        placeholder="experiment directory or trajectory CSV  (empty: latest experiment)")
            yield Button("Open", id="open", compact=True)
        yield Static(id="source-info", classes="context")
        with Horizontal(id="sensors-body"):
            with VerticalScroll(id="summary-pane"):
                yield Static(id="summary")
            with Vertical(id="spark-pane"):
                for key, label in SPARKS:
                    yield Static(label, classes="spark-label", id=f"label-{key}", markup=key != "speed")
                    yield (VelocitySparkline if key == "speed" else Sparkline)([], id=f"spark-{key}")
        yield Footer()

    def on_screen_resume(self) -> None:
        self.query_one("#source", Input).value = str(self.path) if self.path else ""
        self.load(self.path)

    def show(self, path: Path | None) -> None:
        self.path = path

    @on(Button.Pressed, "#open")
    @on(Input.Submitted, "#source")
    def open_source(self) -> None:
        text = self.query_one("#source", Input).value.strip().strip('"')
        self.path = Path(text).expanduser() if text else None
        self.load(self.path)

    def empty(self, message: str) -> None:
        self.query_one("#source-info", Static).update(Text(message, style=DIM))
        self.query_one("#summary", Static).update("")
        for key, label in SPARKS:
            if key == "speed":
                spark = self.query_one("#spark-speed", VelocitySparkline)
                spark.set_velocity([])
                spark.tooltip = None
            else:
                self.query_one(f"#spark-{key}", Sparkline).data = []
            self.query_one(f"#label-{key}", Static).update(label)
        self.query_one("#label-sigma", Static).display = True
        self.query_one("#spark-sigma", Sparkline).display = True

    def load(self, path: Path | None) -> None:
        p = self.project
        origin = ""
        if path is None:
            records = list_experiments(p.experiments_dir)
            if not records:
                self.empty("No experiments recorded yet. Run one from Autonomous Rendezvous.")
                return
            path, origin = records[0].path, "latest experiment · "
        csv_path = resolve_csv(path)
        try:
            trajectory = load(csv_path)
        except (OSError, csv.Error, UnicodeDecodeError) as error:
            self.empty(f"cannot read {csv_path}: {error}")
            return
        snapshot = read_snapshot(csv_path.parent / "configuration.ini")
        external = snapshot is not None and snapshot.controller.type == EXTERNAL_CONTROLLER
        if snapshot is not None:
            rail, ood, limits = snapshot.rail, snapshot.controller.ood_warn_z, "limits from the run's configuration.ini"
        else:
            rail, ood = p.profile.rail, p.profile.controller.ood_warn_z
            limits = "no configuration.ini beside the CSV; limits from the active configuration"
        self.query_one("#source-info", Static).update(
            Text(f"{origin}{p.relative(csv_path)} · {limits}", style=DIM))

        groups: dict[str, list[tuple[str, str, str]]] = {}
        for metric in summarize(trajectory, rail, ood):
            groups.setdefault(metric.group, []).append((metric.name, metric.value, metric.note))
        self.query_one("#summary", Static).update(
            Group(*(section(name, kv_table(rows)) for name, rows in groups.items())) if groups
            else Text("The CSV has no data rows.", style=DIM))

        series = Series()
        series.extend(trajectory.rows)
        run = trajectory.run_rows
        velocity = series.velocity
        if external:
            encoder = EncoderSpeed()
            velocity = []
            for row in trajectory.rows:
                encoder.extend([row])
                if encoder.components is not None:
                    velocity.append(encoder.components)
        self.query_one("#label-sigma", Static).display = not external
        self.query_one("#spark-sigma", Sparkline).display = not external
        data = {
            "distance": series.distance,
            "track": [v * 1e3 for v in trajectory.tracking_errors],
            "age": [abs(v) * 1e3 for r in run if (v := num(r, "meas_age_s")) is not None],
            "sigma": [v for r in run if (v := num(r, "obs_max_z")) is not None],
        }
        for key, label in SPARKS:
            if key == "speed":
                frame = "Rail encoder" if external else "Hill"
                spark = self.query_one("#spark-speed", VelocitySparkline)
                spark.set_velocity(velocity)
                spark.tooltip = f"{frame} velocity X / Y / Z in m/s. Each row has its own scale."
                self.query_one("#label-speed", Static).update(f"{frame} velocity XYZ [m/s]")
                continue
            values = data[key]
            self.query_one(f"#spark-{key}", Sparkline).data = values
            span = f"   {min(values):.3g} … {max(values):.3g}" if values else "   no data"
            self.query_one(f"#label-{key}", Static).update(label + span)
