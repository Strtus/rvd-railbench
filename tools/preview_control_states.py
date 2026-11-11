"""Run the actual TUI and PFCR simulator, with control-border preview enabled.

This launcher has no rail-node connection. All controller commands go to the
C++ ideal simulator, and experiment metadata continues to say hardware=false.
"""
from __future__ import annotations

import argparse
import asyncio
import csv
from datetime import datetime
import json
import os
from pathlib import Path
import time

from textual import events
from textual.binding import Binding
from textual.widgets import Input, Select

from rvdlab.app import RailControlState, RvdLabApp
from rvdlab.screens.rendezvous import RendezvousScreen


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "out/control-status-demo"


class DemoScreen(RendezvousScreen):
    TITLE_TEXT = "LOCAL SIMULATION / CONTROL STATUS DEMO"
    BINDINGS = [Binding("f5", "run", "Replay"), Binding("f6", "stop", "Stop"),
                Binding("escape", "app.quit", "Exit")]

    def apply_defaults(self) -> None:
        super().apply_defaults()
        self.set_start((18.0, 0.3, -0.2))
        self.query_one("#mode", Select).value = "ideal"
        self.query_one("#clock", Select).value = "fast" if self.app.check else "real"
        self.query_one("#max-steps", Input).value = "40"
        self.query_one("#seed", Input).value = "12345"
        self.query_one("#fault", Select).value = "send"
        self.query_one("#fault-step", Input).value = "30"

    def reset(self, directory: Path) -> None:
        super().reset(directory)
        # Exercise the same presentation path as node runs. Execution still goes
        # through the local simulator: no bridge, node, or hardware commands.
        self.hardware_run = True
        self.lab.set_rail_control_state("running")
        self.message = "Simulated send failure at k = 30"
        self.render_status()

    def finish(self, directory: Path, code: int | None) -> None:
        super().finish(directory, code)
        self.app.write_status()


class DemoApp(RvdLabApp):
    CSS_PATH = ROOT / "tui/rvdlab/rvdlab.tcss"
    TITLE = "RVDLAB - SIMULATED RAIL DEMO"

    def __init__(self, session: Path, check: bool = False) -> None:
        self.session, self.check = session, check
        self.started = time.monotonic()
        self.phases = []
        super().__init__(root=ROOT, configs=[ROOT / "configs/rail.ini", ROOT / "configs/simulation.ini"],
                         experiments=session / "experiments", profiles=session / "profiles",
                         node=session / "unused-node.json", welcome=False)
        self.node_path = None  # Ignore all saved or environment-supplied rail connections.

    def get_default_screen(self) -> DemoScreen:
        return DemoScreen()

    def on_mount(self, event: events.Mount) -> None:
        event.prevent_default()
        super().on_mount()
        self.write_status()
        if not self.check:
            self.set_timer(2, self.screen.start)

    def set_rail_control_state(self, state: RailControlState) -> None:
        previous = self.rail_control_state
        super().set_rail_control_state(state)
        if previous != state:
            self.phases.append({"elapsed_s": round(time.monotonic() - self.started, 2), "state": state})
            self.write_status()

    def write_status(self) -> None:
        screen = self.screen
        data = {"pid": os.getpid(), "simulation_only": True, "node_connected": False,
                "state": self.rail_control_state, "phases": self.phases,
                "outcome": getattr(screen, "outcome", None),
                "experiment": str(getattr(screen, "experiment", ""))}
        pending = self.session / "status.tmp"
        pending.write_text(json.dumps(data, indent=2), encoding="utf-8")
        pending.replace(self.session / "status.json")


async def check_demo(app: DemoApp) -> None:
    async with app.run_test(size=(150, 50)) as pilot:
        await pilot.pause()
        screen = app.screen
        screen.start()
        for _ in range(150):
            await pilot.pause(0.1)
            if screen.outcome is not None:
                break
        assert screen.outcome == ("FAULT", "SEND_FAILED"), screen.outcome
        assert app.rail_control_state == "fault"
        # Finish the offline comparison before closing the test's event loop.
        await app.workers.wait_for_complete()
        metadata = json.loads((screen.experiment / "metadata.json").read_text())
        assert metadata["hardware"] is False
        with (screen.experiment / "trajectory.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        cbf = [int(row["k"]) for row in rows if row["intervened"] == "1" and row["status"] == "RUN"]
        assert cbf and cbf[0] == 11, cbf
        assert rows[0]["intervened"] == "0"
        assert rows[-1]["k"] == "30" and rows[-1]["reason"] == "SEND_FAILED"
        report = {"normal_until_step": 10, "cbf_steps": cbf, "fault_step": 30,
                  "outcome": screen.outcome, "hardware": False}
        (app.session / "check.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="verify the simulation headlessly at full speed")
    args = parser.parse_args()
    os.environ.pop("NO_COLOR", None)
    os.environ["COLORTERM"] = "truecolor"
    session = OUTPUT / (("check-" if args.check else "session-") + datetime.now().strftime("%Y%m%d-%H%M%S-%f"))
    session.mkdir(parents=True)
    app = DemoApp(session, check=args.check)
    if args.check:
        asyncio.run(check_demo(app))
    else:
        app.run()


if __name__ == "__main__":
    main()
