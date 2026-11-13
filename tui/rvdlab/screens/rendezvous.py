from __future__ import annotations

import asyncio
import json
import os
import tempfile
import threading
from pathlib import Path
from typing import Any

from rich.table import Table
from rich.text import Text
from textual import on, work
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.widgets import Button, Footer, Input, RichLog, Select, Sparkline, Static

from ..config import Vec3, envelope_violations, hill_to_rail, rail_to_hill, simulation_override
from ..comparison import Comparison, coordinates
from ..experiments import new_experiment_dir, read_experiment, simulation_command
from ..plot import FILTERED, HIGHLIGHT, NOMINAL, RAW, TRACE, PlotView, Trace
from ..project import ProjectError
from ..remote import AXES, Bridge, connect, move, remote_override
from ..telemetry import CsvTail, Series, norm3, num
from .base import DIM, Page
from .node_view import ConfirmModal

NUMBER = r"[-+0-9.eE]*"


def _range(values: list[float]) -> str:
    return f"{min(values):.3g} … {max(values):.3g}" if values else ""


class RendezvousScreen(Page):
    TITLE_TEXT = "Autonomous Rendezvous"
    BINDINGS = [*Page.BINDINGS, Binding("f5", "run", "Run"), Binding("f6", "stop", "Stop"),
                Binding("f9", "rail_stop", "Stop rail")]

    def compose(self) -> ComposeResult:
        yield from self.compose_title()
        yield Static(id="run-context", classes="context")
        with Horizontal(id="run-body"):
            with Vertical(id="run-side"):
                with Horizontal(classes="form-row"):
                    yield Static("rail", classes="form-key mid")
                    node = [("rail node", "node"), ("rail node, z simulated", "node-xy")]
                    targets = [("simulator", "sim")] + (node if self.lab.node_path else [])
                    yield Select(targets, id="target", allow_blank=False, compact=True)
                yield Static("START  ·  HILL FRAME [m]", classes="form-title")
                with Horizontal(classes="form-row"):
                    for axis in "xyz":
                        yield Static(axis, classes="form-key")
                        yield Input(id=f"start-{axis}", classes="num", compact=True, restrict=NUMBER)
                with Horizontal(classes="form-row"):
                    yield Static("mode", classes="form-key mid")
                    yield Select([("ideal", "ideal"), ("bounded", "bounded"), ("plc", "plc")], id="mode",
                                 allow_blank=False, compact=True)
                    yield Static("clock", classes="form-key mid gap")
                    yield Select([("fast", "fast"), ("1 s real", "real")], id="clock",
                                 allow_blank=False, compact=True)
                with Horizontal(classes="form-row"):
                    yield Static("steps", classes="form-key mid")
                    yield Input(id="max-steps", classes="num", compact=True, placeholder="all", restrict=r"[0-9]*")
                    yield Static("seed", classes="form-key mid gap")
                    yield Input(id="seed", classes="num", compact=True, restrict=r"[0-9]*")
                yield Static("FAULT INJECTION", classes="form-title")
                with Horizontal(classes="form-row"):
                    yield Static("fault", classes="form-key mid")
                    yield Select([("none", "none"), ("frozen sensor", "freeze"), ("repeated pose", "repeat"),
                                  ("read failure", "read"), ("send failure", "send")], id="fault",
                                 allow_blank=False, compact=True)
                    yield Static("at k", classes="form-key mid gap")
                    yield Input("30", id="fault-step", classes="num", compact=True, restrict=r"[0-9]*")
                with Horizontal(classes="button-row"):
                    yield Button("Run", id="run", compact=True)
                    yield Button("Stop", id="stop", compact=True, disabled=True)
                    yield Button("To start", id="to-start", compact=True, disabled=True)
                yield Static(id="run-status")
                yield Static("Comparison available after run", id="comparison-status")
                yield Static("distance [m]", classes="spark-label", id="label-distance")
                yield Sparkline([], id="spark-distance")
                yield Static("speed [m/s]", classes="spark-label", id="label-speed")
                yield Sparkline([], id="spark-speed")
            with Vertical(id="plots"):
                with Horizontal(id="plot-legend-row"):
                    with Vertical(id="global-legend"):
                        yield Static("FULL TRAJECTORY", classes="plot-section-title")
                        yield Static("● Current     ◎ Target     ○ Selected", classes="plot-note")
                    with Vertical(id="detail-legend"):
                        with Horizontal(id="event-controls"):
                            yield Static("LOCAL DETAIL", classes="plot-section-title", id="local-title")
                            yield Select([], id="comparison-event", prompt="after run", compact=True)
                        yield Static("8 s prediction · commands held · ○ start", id="local-note")
                for plane, axis in (("xy", "y"), ("xz", "z")):
                    with Horizontal(classes="comparison-row"):
                        yield PlotView(f"Full path · x–{axis}", id=f"plot-{plane}", classes="global-plot",
                                       invert_x=True, x_name="x [m]", y_name=axis)
                        yield PlotView(f"Local · x–{axis}", id=f"local-{plane}", classes="local-plot",
                                       invert_x=True, x_name="x [m]", y_name=axis)
                with Horizontal(classes="chart-legend-row"):
                    legend = Text("─ Actual    ", style=TRACE)
                    legend.append("× CBF    ", style=HIGHLIGHT)
                    legend.append("┄ No CBF (sim)", style=NOMINAL)
                    yield Static(legend, classes="global-chart-legend")
                    local_legend = Text("─ Actual    ", style=TRACE)
                    local_legend.append("─ Raw    ", style=RAW)
                    local_legend.append("─ Filtered", style=FILTERED)
                    yield Static(local_legend, classes="local-chart-legend")
        yield RichLog(id="run-log", max_lines=400, min_width=20, wrap=True)
        yield Footer()

    def on_mount(self) -> None:
        self.bridge: Bridge | None = None
        self.cancel_move: threading.Event | None = None
        self.process: asyncio.subprocess.Process | None = None
        self.running = False
        self.hardware_run = False
        self.stop_requested = False
        self.series = Series()
        self.comparison = Comparison()
        self.comparison_busy = False
        self.last_row: dict[str, str] | None = None
        self.interventions = 0
        self.experiment: Path | None = None
        self.outcome: tuple[str, str] | None = None
        self.message = ""
        self.defaults_from: tuple = ()
        self.apply_defaults()
        self.render_status()

    def apply_defaults(self) -> None:
        """Form defaults from the active configuration; re-applied when the rail profile changes."""
        p = self.project
        self.defaults_from = (p.rail_profile, tuple(p.profile.files))
        sim = p.profile.simulation
        self.set_start(sim.start_hill_m)
        self.query_one("#mode", Select).value = sim.mode
        self.query_one("#clock", Select).value = "fast" if sim.fast else "real"
        self.query_one("#max-steps", Input).value = str(sim.max_steps) if sim.max_steps >= 0 else ""
        self.query_one("#seed", Input).value = str(sim.seed)

    def on_screen_resume(self) -> None:
        p = self.project
        if not self.running and self.defaults_from != (p.rail_profile, tuple(p.profile.files)):
            self.apply_defaults()
        if self.lab.start_override is not None:
            self.set_start(self.lab.start_override)
            self.lab.start_override = None
        if p.build.rail_main is None:
            context = "rail_main is not built: cmake --preset release && cmake --build --preset release"
        elif self.on_node():
            controller = f"{p.profile.controller.type} {p.profile.controller.backend}"
            sim = self.simulated_axes()
            rail = f"REAL RAIL x y, {' '.join(sim)} simulated" if sim else "REAL RAIL"
            context = (f"{rail} through the rail node · {controller} · starts where the rail is · "
                       f"results in {p.relative(p.experiments_dir)}")
        else:
            source = (f"rail profile {p.rail_profile}" if p.rail_profile
                      else " + ".join(f.name for f in p.profile.files) or "built-in defaults")
            context = (f"simulation only · {p.profile.controller.type} {p.profile.controller.backend} · "
                       f"{source} · results in {p.relative(p.experiments_dir)}")
        self.query_one("#run-context", Static).update(Text(context, style=DIM))

    def on_unmount(self) -> None:
        if self.process is not None and self.process.returncode is None:
            self.process.terminate()

    def set_start(self, start: Vec3) -> None:
        for axis, value in zip("xyz", start):
            self.query_one(f"#start-{axis}", Input).value = f"{value:g}"

    def fail(self, message: str) -> None:
        self.message = message
        self.render_status()
        self.notify(message, severity="error")

    def action_run(self) -> None:
        self.start()

    def action_stop(self) -> None:
        self.stop()

    @on(Button.Pressed, "#run")
    def _run_pressed(self) -> None:
        self.start()

    @on(Button.Pressed, "#stop")
    def _stop_pressed(self) -> None:
        self.stop()

    def on_node(self) -> bool:
        return self.query_one("#target", Select).value in ("node", "node-xy")

    def simulated_axes(self) -> tuple[str, ...]:
        return ("z",) if self.query_one("#target", Select).value == "node-xy" else ()

    @on(Select.Changed, "#target")
    def target_changed(self) -> None:
        self.set_controls(self.running)
        self.on_screen_resume()

    def start(self) -> None:
        if self.running:
            return
        p = self.project
        if p.config_error:
            self.fail(f"configuration error: {p.config_error}")
            return
        if self.on_node():
            self.start_on_node()
            return
        try:
            start = tuple(float(self.query_one(f"#start-{a}", Input).value) for a in "xyz")
        except ValueError:
            self.fail("start position needs three numbers")
            return
        violations = envelope_violations(p.profile.rail, hill_to_rail(p.profile.rail, start))  # type: ignore[arg-type]
        if violations:
            self.fail("start maps outside the rail envelope: " + "; ".join(violations))
            return
        steps = self.query_one("#max-steps", Input).value.strip()
        seed_text = self.query_one("#seed", Input).value.strip()
        seed = int(seed_text) if seed_text else p.profile.simulation.seed
        if seed > 2**32 - 1:
            self.fail("seed must fit in 32 bits")
            return
        mode = str(self.query_one("#mode", Select).value)
        fast = self.query_one("#clock", Select).value == "fast"
        fault = str(self.query_one("#fault", Select).value)
        fault_text = self.query_one("#fault-step", Input).value.strip()
        if fault != "none" and not fault_text:
            self.fail("fault injection needs a step")
            return
        at = int(fault_text) if fault != "none" else -1
        faults = {"freeze_at": at if fault == "freeze" else -1, "repeat_pose_at": at if fault == "repeat" else -1,
                  "fail_read_at": at if fault == "read" else -1, "fail_send_at": at if fault == "send" else -1}
        directory = new_experiment_dir(p.experiments_dir)
        handle, name = tempfile.mkstemp(prefix="rvdlab-", suffix=".ini")
        with os.fdopen(handle, "w", encoding="utf-8") as stream:
            stream.write(simulation_override(start, mode, fast, int(steps) if steps else -1, seed,  # type: ignore[arg-type]
                                             **faults))
        override = Path(name)
        try:
            args = simulation_command(p, override, directory)
        except ProjectError as error:
            override.unlink(missing_ok=True)
            self.fail(str(error))
            return
        p.experiments_dir.mkdir(parents=True, exist_ok=True)
        self.reset(directory)
        self.execute(args, directory, override)

    def stop(self) -> None:
        if self.bridge is not None:
            if self.stop_requested and self.process is not None and self.process.returncode is None:
                self.process.terminate()  # second press: rail_main did not end on its own
                return
            self.stop_requested = True
            self.stop_bridge(self.bridge)
            return
        if self.process is not None and self.process.returncode is None:
            self.stop_requested = True
            self.process.terminate()

    def action_rail_stop(self) -> None:
        if self.cancel_move is not None:
            self.cancel_move.set()
        if self.bridge is not None:
            self.stop()
        elif self.lab.node_path is not None:
            self.stop_bridge(None)

    @work(thread=True, group="rail-stop")
    def stop_bridge(self, bridge: Bridge | None) -> None:
        try:
            if bridge is not None:
                bridge.stop()
            else:
                connect(self.lab.node_path).stop()  # type: ignore[arg-type]
            message = "rail stopped"
        except Exception as error:
            message = f"STOP FAILED: {error}. Use the emergency stop."
        self.app.call_from_thread(self.log_line, message)

    def log_line(self, text: str) -> None:
        self.query_one("#run-log", RichLog).write(text)

    # runs on the rail node

    def node_state(self) -> tuple[Any, dict[str, Any]]:
        node = connect(self.lab.node_path)  # type: ignore[arg-type]
        return node, node.state()

    def start_on_node(self) -> None:
        if self.project.build.rail_main is None:
            self.fail("rail_main is not built")
            return
        steps = self.query_one("#max-steps", Input).value.strip()
        self.prepare_node_run(int(steps) if steps else -1, self.simulated_axes())

    @work(thread=True, exclusive=True, group="node-run")
    def prepare_node_run(self, steps: int, simulated: tuple[str, ...] = ()) -> None:
        try:
            _, state = self.node_state()
        except Exception as error:
            self.app.call_from_thread(self.fail, f"rail node: {error}")
            return
        here = tuple(state["position"][a] for a in AXES)
        hill = rail_to_hill(self.project.profile.rail, here)  # type: ignore[arg-type]
        text = ("PFCR will drive the REAL RAIL from where it stands now.\n"
                f"rail  {here[0]:.4f}  {here[1]:.4f}  {here[2]:.4f} m\n"
                f"Hill  {hill[0]:.3f}  {hill[1]:.3f}  {hill[2]:.3f} m\n"
                f"steps {steps if steps >= 0 else 'until docked or stopped'}\n"
                + (f"simulated {' '.join(simulated)}: that rail axis stays where it is\n" if simulated else "")
                + "\n"
                "Watch the rail and keep the emergency stop within reach. F6 or F9 stops it.")
        self.app.call_from_thread(self.app.push_screen, ConfirmModal("Run on the rail", text, "Run"),
                                  lambda ok: self.arm_node(steps, simulated) if ok else None)

    @work(thread=True, exclusive=True, group="node-run")
    def arm_node(self, steps: int, simulated: tuple[str, ...] = ()) -> None:
        try:
            bridge = Bridge(connect(self.lab.node_path),  # type: ignore[arg-type]
                            log=lambda m: self.app.call_from_thread(self.log_line, m), simulated=simulated,
                            on_fault=lambda: self.app.call_from_thread(self.mark_control_fault))
            bridge.acquire()
        except Exception as error:
            self.app.call_from_thread(self.fail, f"cannot take control of the rail: {error}")
            return
        bridge.start()
        self.app.call_from_thread(self.launch_node_run, bridge, steps)

    def launch_node_run(self, bridge: Bridge, steps: int) -> None:
        p = self.project
        directory = new_experiment_dir(p.experiments_dir)
        handle, name = tempfile.mkstemp(prefix="rvdlab-", suffix=".ini")
        with os.fdopen(handle, "w", encoding="utf-8") as stream:
            stream.write(remote_override(steps))
        override = Path(name)
        args = simulation_command(p, override, directory) + ["--bridge", str(bridge.port)]
        bridge.record(directory / "sensors.csv")
        p.experiments_dir.mkdir(parents=True, exist_ok=True)
        self.bridge = bridge
        self.reset(directory)
        self.execute(args, directory, override)

    @on(Button.Pressed, "#to-start")
    def to_start(self) -> None:
        try:
            start = tuple(float(self.query_one(f"#start-{a}", Input).value) for a in "xyz")
        except ValueError:
            self.fail("start position needs three numbers")
            return
        rail = self.project.profile.rail
        goal = hill_to_rail(rail, start)  # type: ignore[arg-type]
        violations = envelope_violations(rail, goal)
        if violations:
            self.fail("start maps outside the rail envelope: " + "; ".join(violations))
            return
        speed = {a: 0.5 * v for a, v in zip(AXES, rail.velocity_m_s)}
        text = (f"Move the REAL RAIL to the start Hill ({start[0]:g}, {start[1]:g}, {start[2]:g}) m,\n"
                f"rail ({goal[0]:.4f}, {goal[1]:.4f}, {goal[2]:.4f}) m, at half the rail speed limits.\n\n"
                "Watch the rail and keep the emergency stop within reach. F9 stops it.")
        self.app.push_screen(ConfirmModal("Move to start", text, "Move"),
                             lambda ok: self.move_to_start(dict(zip(AXES, goal)), speed) if ok else None)

    @work(thread=True, exclusive=True, group="node-move")
    def move_to_start(self, goal: dict[str, float], speed: dict[str, float]) -> None:
        self.cancel_move = threading.Event()
        self.app.call_from_thread(self.log_line, "moving to the start ...")
        try:
            status = move(connect(self.lab.node_path), goal, speed, self.cancel_move)  # type: ignore[arg-type]
        except Exception as error:
            status = f"failed: {error}"
        self.cancel_move = None
        self.app.call_from_thread(self.log_line, f"move to start {status}")

    def reset(self, directory: Path) -> None:
        self.workers.cancel_group(self, "comparison")
        self.comparison = Comparison()
        self.comparison_busy = False
        self.query_one("#comparison-event", Select).set_options([])
        self.query_one("#comparison-status", Static).update("Comparison available after run")
        self.running = True
        self.hardware_run = self.bridge is not None
        self.lab.set_rail_control_state("running" if self.hardware_run else "idle")
        self.stop_requested = False
        self.series = Series()
        self.last_row = None
        self.interventions = 0
        self.experiment = directory
        self.outcome = None
        self.message = ""
        self.query_one("#run-log", RichLog).clear()
        self.refresh_plots()
        self.set_controls(True)
        self.render_status()

    def set_controls(self, running: bool) -> None:
        node = self.on_node()
        self.query_one("#run", Button).disabled = running
        self.query_one("#stop", Button).disabled = not running
        self.query_one("#to-start", Button).disabled = running or not node
        for widget in self.query("#run-side Input, #run-side Select"):
            widget.disabled = running
        if not running and node:
            for name in ("#mode", "#clock", "#seed", "#fault", "#fault-step"):
                self.query_one(name).disabled = True

    @work(exclusive=True, group="simulation")
    async def execute(self, args: list[str], directory: Path, override: Path) -> None:
        log = self.query_one("#run-log", RichLog)
        shown = [self.project.relative(Path(a)) if os.path.isabs(a) else a for a in args]
        log.write(Text("$ " + " ".join(shown), style=DIM))
        tail = CsvTail(directory / "trajectory.csv")
        timer = self.set_interval(0.25, lambda: self.consume(tail.poll()))
        code = None
        try:
            self.process = await asyncio.create_subprocess_exec(
                *args, cwd=str(self.project.root), stdin=asyncio.subprocess.DEVNULL,
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
            assert self.process.stdout is not None
            async for raw in self.process.stdout:
                log.write(raw.decode("utf-8", "replace").rstrip())
            code = await self.process.wait()
            if code not in (0, 1) and not self.stop_requested:
                self.mark_control_fault()
        except OSError as error:
            log.write(f"cannot start rail_main: {error}")
            self.mark_control_fault()
        except asyncio.CancelledError:
            if self.process is not None and self.process.returncode is None:
                self.process.terminate()
            if not self.stop_requested:
                self.mark_control_fault()
            raise
        finally:
            timer.stop()
            # Consume a terminal fault before waiting for bridge cleanup.
            self.consume(tail.poll())
            override.unlink(missing_ok=True)
            self.process = None
            if self.bridge is not None:
                bridge, self.bridge = self.bridge, None
                await asyncio.to_thread(bridge.close)
                if directory.is_dir():
                    (directory / "rail_node.json").write_text(json.dumps(bridge.describe(), indent=1) + "\n")
        self.finish(directory, code)

    def mark_control_fault(self) -> None:
        if self.hardware_run:
            self.lab.set_rail_control_state("fault")

    def consume(self, rows: list[dict[str, str]]) -> None:
        if not rows:
            return
        self.series.extend(rows)
        self.interventions += sum(1 for r in rows if r.get("intervened") == "1")
        self.last_row = rows[-1]
        if self.hardware_run and self.running:
            for row in rows:
                reason = row.get("reason")
                expected_stop = self.stop_requested and reason == "STOP_REQUESTED"
                if row.get("status") in ("ABORT", "FAULT") and reason != "MAX_STEPS" and not expected_stop:
                    self.mark_control_fault()
            if self.lab.rail_control_state != "fault" and self.last_row.get("status") == "RUN":
                self.lab.set_rail_control_state(
                    "intervening" if self.last_row.get("intervened") == "1" else "running")
        self.refresh_plots()
        self.render_status()

    def finish(self, directory: Path, code: int | None) -> None:
        record = read_experiment(directory)
        if self.stop_requested:
            self.outcome = ("INTERRUPTED", "stopped from the interface")
        elif record is not None and record.outcome != "INCOMPLETE":
            self.outcome = (record.outcome, record.reason)
            if record.interventions is not None:
                self.interventions = record.interventions
        elif code is None:
            self.outcome = ("NOT STARTED", "")
        else:
            self.outcome = ("INCOMPLETE", f"exit code {code}")
        if self.hardware_run:
            expected_stop = self.stop_requested and record is not None and record.reason == "STOP_REQUESTED"
            if ((record is not None and record.outcome == "FAULT" and not expected_stop)
                    or self.outcome[0] not in ("DOCKED", "LIMIT_REACHED", "INTERRUPTED")):
                self.mark_control_fault()
            if self.lab.rail_control_state != "fault":
                self.lab.set_rail_control_state("idle")
        self.running = False
        self.set_controls(False)
        self.render_status()
        self.notify(" ".join(self.outcome).strip(), title="Run finished")
        if self.series.x and (directory / "configuration.ini").is_file():
            self.comparison_busy = True
            self.prepare_comparison(directory, self.project.policy.directory)

    @work(exclusive=True, group="comparison")
    async def prepare_comparison(self, directory: Path, model: Path) -> None:
        process = None
        status = self.query_one("#comparison-status", Static)
        status.update("Computing offline comparison ...")
        try:
            replay = self.project.build.rail_replay
            if replay is None:
                raise RuntimeError("rail_replay is not built")
            parent = directory.parent / "comparisons"
            parent.mkdir(exist_ok=True)
            output = parent / directory.name
            process = await asyncio.create_subprocess_exec(
                str(replay), "--preview", str(directory), str(model), str(output),
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
            stdout, _ = await asyncio.wait_for(process.communicate(), timeout=30)
            if process.returncode:
                raise RuntimeError(stdout.decode("utf-8", "replace").strip())
            comparison = Comparison.load(output, directory)
            if self.experiment != directory:
                return
            self.comparison = comparison
            select = self.query_one("#comparison-event", Select)
            select.set_options([(f"k = {k}", k) for k in comparison.events])
            event = comparison.strongest_event()
            if event is not None:
                select.value = event
            last = comparison.nominal[-1]
            outcome = last['status'].replace('_', ' ').lower()
            status.update(f"No CBF: {outcome} · k={last['k']}\nModel only · no rail/noise")
            self.refresh_plots()
        except (OSError, ValueError, KeyError, RuntimeError, asyncio.TimeoutError) as error:
            if self.experiment == directory:
                status.update(f"Comparison unavailable: {error}")
        finally:
            if process is not None and process.returncode is None:
                process.kill()
                await process.wait()
            if self.experiment == directory:
                self.comparison_busy = False

    @on(Select.Changed, "#comparison-event")
    def comparison_event_changed(self) -> None:
        if hasattr(self, "series"):
            self.refresh_plots()

    def refresh_local_plots(self) -> None:
        event = self.query_one("#comparison-event", Select).value
        comparison = getattr(self, "comparison", Comparison())
        rows = comparison.events.get(event, [])
        actual = [r for r in comparison.actual if rows and int(event) <= int(r["k"]) <= int(event) + 8]
        for plane, axis in (("xy", "y"), ("xz", "z")):
            overlays = []
            if rows:
                for prefix, style in (("raw", RAW), ("filtered", FILTERED)):
                    xs, ys = coordinates(rows, f"{prefix}_x", f"{prefix}_{axis}")
                    overlays.append(Trace(xs, ys, style))
            xs, ys = coordinates(actual, "hx", f"h{axis}")
            markers = [(xs[0], ys[0], "○")] if xs else []
            self.query_one(f"#local-{plane}", PlotView).plot(xs, ys, markers, overlays=overlays)

    def refresh_plots(self) -> None:
        s = self.series
        if s.x:
            for name, values in (("xy", s.y), ("xz", s.z)):
                highlights = [(x, y) for x, y, active in zip(s.x, values, s.intervened) if active]
                nx, ny = coordinates(self.comparison.nominal, "hx", "hy" if name == "xy" else "hz")
                overlays = [Trace(nx, ny, NOMINAL, dashed=True)] if nx else []
                markers = [(0.0, 0.0, "◎")]
                event = self.query_one("#comparison-event", Select).value
                selected = self.comparison.events.get(event, [])
                if selected:
                    axis = "y" if name == "xy" else "z"
                    markers.append((float(selected[0]["raw_x"]), float(selected[0][f"raw_{axis}"]), "○"))
                markers.append((s.x[-1], values[-1], "●"))
                self.query_one(f"#plot-{name}", PlotView).plot(
                    s.x, values, markers,
                    highlights=highlights, overlays=overlays)
        else:
            self.query_one("#plot-xy", PlotView).plot([], [])
            self.query_one("#plot-xz", PlotView).plot([], [])
        self.query_one("#spark-distance", Sparkline).data = list(s.distance)
        self.query_one("#spark-speed", Sparkline).data = list(s.speed)
        self.query_one("#label-distance", Static).update(f"distance [m]   {_range(s.distance)}")
        self.query_one("#label-speed", Static).update(f"speed [m/s]   {_range(s.speed)}")
        self.refresh_local_plots()
        plots = list(self.query("#plots PlotView"))
        label_width = max(plot.axis_label_width for plot in plots)
        for plot in plots:
            plot.align_axis_labels(label_width)

    def render_status(self) -> None:
        row = self.last_row
        if self.running:
            state = "running" + (f" · k {row.get('k')}" if row else "")
        elif self.outcome:
            state = "finished"
        else:
            state = "idle"
        rows = [("State", state)]
        if row:
            h = [num(row, n) for n in ("hx", "hy", "hz")]
            if None not in h:
                rows.append(("Hill", "  ".join(f"{v:.3f}" for v in h) + " m"))  # type: ignore[str-format]
            d, v = norm3(row, "hx", "hy", "hz"), norm3(row, "hvx", "hvy", "hvz")
            if d is not None:
                rows.append(("Distance", f"{d:.3f} m"))
            if v is not None:
                rows.append(("Speed", f"{v:.3f} m/s"))
            rows.append(("Safety", f"{self.interventions} interventions"))
        if self.outcome:
            rows.append(("Outcome", " · ".join(part for part in self.outcome if part)))
        if self.experiment is not None:
            rows.append(("Experiment", self.experiment.name))
        table = Table(box=None, show_header=False, pad_edge=False, padding=(0, 2, 0, 0))
        table.add_column(style=DIM, no_wrap=True)
        table.add_column()
        for name, value in rows:
            table.add_row(name, value)
        if self.message:
            table.add_row("Note", Text(self.message, style="bold"))
        self.query_one("#run-status", Static).update(table)
