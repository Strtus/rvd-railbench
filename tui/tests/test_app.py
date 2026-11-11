import io
import json

import pytest
from rich.console import Console
from textual.widgets import Input, Select, Static

from rvdlab.app import RvdLabApp
from rvdlab.profiles import read_active
from rvdlab.project import find_build

from conftest import ROOT

BUILT = find_build(ROOT).rail_main is not None
PAGES = (("1", "OverviewScreen"), ("2", "MotionScreen"), ("3", "RendezvousScreen"), ("4", "SensorsScreen"),
         ("5", "ManagerScreen"), ("6", "SetupScreen"))


def text(widget: Static) -> str:
    console = Console(width=200, file=io.StringIO(), record=True)
    console.print(widget.content)
    return console.export_text()


async def wait_for(pilot, condition, seconds: float = 60.0) -> None:
    for _ in range(int(seconds * 10)):
        await pilot.pause(0.1)
        if condition():
            return
    raise AssertionError("condition not reached")


async def test_first_start_opens_rail_setup(tmp_path):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=tmp_path / "profiles")
    async with app.run_test(size=(120, 45)) as pilot:
        await pilot.pause()
        assert type(app.screen).__name__ == "SetupScreen"
        assert app.screen.query_one("#profile-name", Input).value == "my-rail"


async def test_home_shows_version_profile_and_honest_device_status(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles)
    async with app.run_test(size=(110, 40)) as pilot:
        await pilot.pause()
        assert type(app.screen).__name__ == "HomeScreen"
        assert app.project.version in text(app.screen.query_one("#version", Static))
        status = text(app.screen.query_one("#status", Static))
        assert "example-rail" in status
        assert "Connected" not in status and "No driver" in status


async def test_menu_commands_open_each_page_and_escape_returns(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles)
    async with app.run_test(size=(120, 45)) as pilot:
        for key, screen in PAGES:
            await pilot.press(key, "enter")
            await pilot.pause()
            assert type(app.screen).__name__ == screen
            await pilot.press("escape")
            await pilot.pause()
            assert type(app.screen).__name__ == "HomeScreen"
        # the last opened item stays selected; down wraps from the last item to the first
        await pilot.press("down", "enter")
        await pilot.pause()
        assert type(app.screen).__name__ == "OverviewScreen"


async def test_unknown_command_is_reported(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles)
    async with app.run_test(size=(110, 40)) as pilot:
        await pilot.press("x", "enter")
        await pilot.pause()
        assert "unknown command" in text(app.screen.query_one("#message", Static))


async def test_motion_converter_maps_hill_to_rail(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles)
    async with app.run_test(size=(120, 40)) as pilot:
        await pilot.press("2", "enter")
        await pilot.pause()
        hill_y = app.screen.query_one("#hill-y", Input)
        hill_y.focus()
        hill_y.value = "2"
        await pilot.pause()
        assert float(app.screen.query_one("#rail-y", Input).value) == pytest.approx(0.2)


async def test_setup_saves_a_new_profile_and_makes_it_active(tmp_path):
    directory = tmp_path / "profiles"
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=directory)
    async with app.run_test(size=(120, 45)) as pilot:
        await pilot.pause()
        screen = app.screen
        screen.query_one("#profile-name", Input).value = "short-rail"
        screen.query_one("#max-x", Input).value = "6"
        screen.query_one("#start-x", Input).value = "5"
        screen.action_save()
        await pilot.pause()
        assert read_active(directory) == "short-rail"
        assert app.project.rail_profile == "short-rail"
        assert app.project.profile.rail.maximum_m[0] == 6.0
        assert type(app.screen).__name__ == "HomeScreen"
        assert "short-rail" in text(app.screen.query_one("#status", Static))


async def test_setup_refuses_a_start_outside_the_rail(tmp_path):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=tmp_path / "profiles")
    async with app.run_test(size=(120, 45)) as pilot:
        await pilot.pause()
        screen = app.screen
        screen.query_one("#start-x", Input).value = "100"
        screen.action_save()
        await pilot.pause()
        assert "outside the envelope" in text(screen.query_one("#setup-result", Static))
        assert read_active(tmp_path / "profiles") is None


@pytest.mark.skipif(not BUILT, reason="rail_main is not built")
async def test_rendezvous_runs_a_bounded_simulation(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles)
    async with app.run_test(size=(140, 45)) as pilot:
        await pilot.press("3", "enter")
        await pilot.pause()
        screen = app.screen
        screen.query_one("#max-steps", Input).value = "12"
        screen.query_one("#clock", Select).value = "fast"
        screen.start()
        assert not app.has_class("rail-control-active")
        await wait_for(pilot, lambda: screen.outcome is not None)
        assert not app.has_class("rail-control-active")
        assert screen.outcome == ("LIMIT_REACHED", "MAX_STEPS")
        assert len(screen.series.x) == 13
        assert json.loads((screen.experiment / "summary.json").read_text())["reason"] == "MAX_STEPS"
        assert json.loads((screen.experiment / "metadata.json").read_text())["hardware"] is False


@pytest.mark.skipif(not BUILT, reason="rail_main is not built")
async def test_fault_injection_trips_the_safety_supervisor(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles)
    async with app.run_test(size=(140, 45)) as pilot:
        await pilot.press("3", "enter")
        await pilot.pause()
        screen = app.screen
        screen.query_one("#clock", Select).value = "fast"
        screen.query_one("#fault", Select).value = "send"
        screen.query_one("#fault-step", Input).value = "5"
        screen.start()
        assert not app.has_class("rail-control-active")
        await wait_for(pilot, lambda: screen.outcome is not None)
        assert screen.outcome == ("FAULT", "SEND_FAILED")


@pytest.mark.skipif(not BUILT, reason="rail_main is not built")
async def test_suggest_fits_the_example_rail_and_validates_it(tmp_path):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=tmp_path / "profiles")
    async with app.run_test(size=(140, 60)) as pilot:
        await pilot.pause()
        screen = app.screen
        screen.suggest()
        await pilot.pause(0.5)
        await wait_for(pilot, lambda: not screen.busy, seconds=120)
        report = text(screen.query_one("#setup-result", Static))
        assert float(screen.query_one("#scale-0", Input).value) == pytest.approx(0.18, abs=0.005)
        assert "PASS" in report and "9/9 docked" in report
