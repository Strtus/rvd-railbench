import asyncio
import json
import threading
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest
from textual.color import Color

from conftest import ROOT
from rvdlab.app import RvdLabApp
from rvdlab.screens.node_view import ConfirmModal


async def wait_for(pilot, condition):
    for _ in range(40):
        await pilot.pause(0.05)
        if condition():
            return
    raise AssertionError("condition not reached")


def border_color(screen):
    edges = [screen.styles.border_top, screen.styles.border_right,
             screen.styles.border_bottom, screen.styles.border_left]
    assert all(edge == edges[0] and edge[0] == "thick" for edge in edges)
    return edges[0][1]


async def test_status_colors_and_fault_pulse_follow_pages_without_layout_jitter(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles)
    async with app.run_test(size=(140, 50)) as pilot:
        await app.push_screen("rendezvous")
        await pilot.pause()
        assert not app.screen.styles.border_top[0]
        app.set_rail_control_state("running")
        await pilot.pause()
        assert border_color(app.screen) == Color.parse("#3ddc84")
        region = app.screen.query_one("#run-body").region
        await pilot.pause(0.6)
        assert border_color(app.screen) == Color.parse("#3ddc84")
        assert app._rail_control_timer is None
        app.set_rail_control_state("intervening")
        await pilot.pause(0.6)
        assert border_color(app.screen) == Color.parse("#ffbf3b")
        assert app._rail_control_timer is None
        assert app.screen.query_one("#run-body").region == region

        app.set_rail_control_state("fault")
        await pilot.pause()
        assert border_color(app.screen) == Color.parse("#ff3b30")
        await wait_for(pilot, lambda: border_color(app.screen) == Color.parse("#651c24"))
        assert app.screen.query_one("#run-body").region == region
        await wait_for(pilot, lambda: border_color(app.screen) == Color.parse("#ff3b30"))
        assert app.screen.query_one("#run-body").region == region

        await pilot.resize_terminal(100, 40)
        await pilot.pause()
        assert app.screen.region.size == (100, 40)
        border_color(app.screen)
        await pilot.press("escape")
        border_color(app.screen)
        await app.push_screen("overview")
        await pilot.pause()
        border_color(app.screen)
        await app.push_screen(ConfirmModal("Test", "UI only", "OK"))
        await pilot.pause()
        border_color(app.screen)

        app.set_rail_control_state("idle")
        await pilot.pause(0.6)
        assert not app.screen.styles.border_top[0]
        await pilot.press("escape", "escape")
        await app.push_screen("rendezvous")
        await pilot.pause()
        assert not app.screen.styles.border_top[0]
        assert not app.has_class("rail-control-bright")


async def test_failed_process_start_flashes_during_and_after_bridge_cleanup(tmp_path, profiles, monkeypatch):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles)
    closing, released = threading.Event(), threading.Event()

    def close():
        closing.set()
        assert released.wait(5), "bridge cleanup was not released"

    async with app.run_test(size=(140, 50)) as pilot:
        await app.push_screen("rendezvous")
        await pilot.pause()
        screen = app.screen
        screen.bridge = SimpleNamespace(close=close)
        directory = tmp_path / "failed-run"
        override = tmp_path / "override.ini"
        override.write_text("")
        monkeypatch.setattr(asyncio, "create_subprocess_exec", AsyncMock(side_effect=OSError("not executable")))
        screen.reset(directory)
        screen.execute(["not-executable"], directory, override)
        try:
            await wait_for(pilot, closing.is_set)
            assert app.rail_control_state == "fault"
            border_color(app.screen)
        finally:
            released.set()
        await wait_for(pilot, lambda: screen.outcome is not None)
        assert screen.outcome == ("NOT STARTED", "")
        assert not override.exists()
        assert app.rail_control_state == "fault"
        border_color(app.screen)


async def test_cbf_uses_the_latest_step_and_faults_latch_until_the_next_run(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles)
    async with app.run_test(size=(140, 50)) as pilot:
        await app.push_screen("rendezvous")
        await pilot.pause()
        screen = app.screen
        screen.bridge = object()  # Mark a hardware run without opening any connection.
        screen.reset(tmp_path / "run")
        assert app.rail_control_state == "running"
        screen.consume([{"status": "RUN", "intervened": "1"}])
        assert app.rail_control_state == "intervening"
        screen.consume([{"status": "RUN", "intervened": "1"}, {"status": "RUN", "intervened": "0"}])
        assert screen.interventions == 2
        assert app.rail_control_state == "running"
        screen.consume([{"status": "ABORT", "reason": "READ_FAILED", "intervened": "1"}])
        assert app.rail_control_state == "fault"
        screen.consume([{"status": "RUN", "intervened": "0"}])
        assert app.rail_control_state == "fault"
        screen.reset(tmp_path / "next-run")
        assert app.rail_control_state == "running" and app._rail_control_timer is None


@pytest.mark.parametrize("outcome,reason,operator_stop,expected", [
    ("DOCKED", "CAPTURED", False, "idle"),
    ("LIMIT_REACHED", "MAX_STEPS", False, "idle"),
    ("FAULT", "STOP_REQUESTED", True, "idle"),
    ("FAULT", "STOP_REQUESTED", False, "fault"),
    ("FAULT", "READ_FAILED", False, "fault"),
    ("FAULT", "READ_FAILED", True, "fault"),
    ("INCOMPLETE", "", False, "fault"),
])
async def test_run_outcome_distinguishes_expected_stops_from_faults(
        tmp_path, profiles, outcome, reason, operator_stop, expected):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles)
    async with app.run_test(size=(140, 50)) as pilot:
        await app.push_screen("rendezvous")
        await pilot.pause()
        screen = app.screen
        screen.bridge = object()
        directory = tmp_path / "run"
        directory.mkdir()
        (directory / "metadata.json").write_text(json.dumps({"hardware": True}))
        (directory / "summary.json").write_text(json.dumps({"outcome": outcome, "reason": reason}))
        screen.reset(directory)
        screen.stop_requested = operator_stop
        if outcome != "INCOMPLETE":
            screen.consume([{"status": "DOCKED" if outcome == "DOCKED" else "ABORT", "reason": reason}])
        screen.finish(directory, 0 if outcome == "DOCKED" else 1)
        assert app.rail_control_state == expected


async def test_simulated_cbf_and_faults_do_not_light_the_border(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles)
    async with app.run_test(size=(140, 50)) as pilot:
        await app.push_screen("rendezvous")
        await pilot.pause()
        screen = app.screen
        screen.reset(tmp_path / "simulation")
        for row in ({"status": "RUN", "intervened": "1"}, {"status": "ABORT", "reason": "READ_FAILED"}):
            screen.consume([row])
            assert app.rail_control_state == "idle"
        screen.finish(tmp_path / "simulation", 2)
        assert app.rail_control_state == "idle" and not app.screen.styles.border_top[0]
