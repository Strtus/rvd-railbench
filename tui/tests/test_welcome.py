import hashlib
import json
from pathlib import Path

import pytest
from textual.widgets import Input

from conftest import ROOT
from rvdlab.__main__ import main
from rvdlab.app import RvdLabApp
from rvdlab.screens.welcome import OrbitalArt, WelcomeScreen
from rvdlab.welcome import DURATION, OrbitalAnimation, mark_welcome_seen, welcome_needed


@pytest.fixture(autouse=True)
def welcome_home(tmp_path, monkeypatch):
    directory = tmp_path / "user-data"
    monkeypatch.setenv("RVDLAB_HOME", str(directory))
    return directory


def test_frames_match_recorded_powershell_output():
    reference = json.loads((Path(__file__).parent / "fixtures" / "welcome_frames.json").read_text(encoding="utf-8"))
    animation = OrbitalAnimation()
    for frame in reference["frames"]:
        text = animation.render(frame["index"])
        assert hashlib.sha256(text.plain.encode("utf-8")).hexdigest() == frame["text_sha256"]
        _, hot = animation.cells(frame["index"])
        assert {str(k): v for k, v in hot.items()} == frame["hot"]


@pytest.mark.parametrize("size", [(76, 18), (40, 10), (16, 4)])
def test_smaller_art_stays_in_bounds(size):
    cols, rows = size
    animation = OrbitalAnimation(cols, rows)
    for frame in (0, 89, 90, 115, 449):
        lines = animation.render(frame).plain.splitlines()
        assert len(lines) == rows
        assert all(len(line) == cols for line in lines)
        assert all(0 <= index < cols * rows for index in animation.cells(frame)[1])


def test_welcome_marker_and_unwritable_directory(welcome_home, monkeypatch):
    assert welcome_needed()
    mark_welcome_seen()
    assert (welcome_home / "welcome-seen").is_file()
    assert not welcome_needed()
    # A path occupied by a file cannot be made into a state directory.
    blocked = welcome_home / "blocked"
    blocked.write_text("file", encoding="utf-8")
    monkeypatch.setenv("RVDLAB_HOME", str(blocked))
    mark_welcome_seen()  # must not block entry to the TUI


@pytest.mark.parametrize("key", ["enter", "escape", "space", "x", "tab"])
async def test_skip_enters_rail_setup_without_typing_into_it(tmp_path, welcome_home, key):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=tmp_path / "profiles", welcome=True)
    async with app.run_test(size=(120, 40)) as pilot:
        assert isinstance(app.screen, WelcomeScreen)
        await pilot.press(key)
        await pilot.pause()
        assert type(app.screen).__name__ == "SetupScreen"
        assert app.screen.query_one("#profile-name", Input).value == "my-rail"
        assert (welcome_home / "welcome-seen").is_file()


async def test_first_interactive_launch_once_then_replay(tmp_path, profiles, monkeypatch):
    # Keep the headless test driver, but exercise the normal interactive startup policy.
    monkeypatch.setattr(RvdLabApp, "is_headless", property(lambda self: False))
    arguments = dict(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles)
    first = RvdLabApp(**arguments)
    async with first.run_test(size=(120, 40)) as pilot:
        assert isinstance(first.screen, WelcomeScreen)
        await pilot.press("1")
        await pilot.pause()
        assert type(first.screen).__name__ == "HomeScreen"
        assert first.screen.query_one("#command", Input).value == ""
    returning = RvdLabApp(**arguments)
    async with returning.run_test() as pilot:
        assert type(returning.screen).__name__ == "HomeScreen"
    replay = RvdLabApp(**arguments, welcome=True)
    async with replay.run_test() as pilot:
        assert isinstance(replay.screen, WelcomeScreen)


async def test_completion_and_resize(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles, welcome=True)
    async with app.run_test(size=(120, 40)) as pilot:
        screen = app.screen
        assert isinstance(screen, WelcomeScreen)
        assert screen.query_one(OrbitalArt).size == (108, 27)
        await pilot.resize_terminal(80, 24)
        await pilot.pause()
        assert screen.query_one(OrbitalArt).size.width <= 76
        screen._started -= DURATION
        screen._tick()
        await pilot.pause()
        assert type(app.screen).__name__ == "HomeScreen"
        assert not welcome_needed()
        # A queued tick after dismissal must not touch the new screen.
        screen._tick()


async def test_explicit_skip_leaves_first_launch_marker_unset(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles, welcome=False)
    async with app.run_test() as pilot:
        assert type(app.screen).__name__ == "HomeScreen"
        assert welcome_needed()


@pytest.mark.parametrize("flags, expected", [([], None), (["--welcome"], True), (["--no-welcome"], False)])
def test_cli_passes_welcome_preference(flags, expected, monkeypatch):
    received = {}

    class FakeApp:
        def __init__(self, **kwargs):
            received.update(kwargs)

        def run(self):
            pass

    monkeypatch.setattr("rvdlab.app.RvdLabApp", FakeApp)
    assert main(flags) == 0
    assert received["welcome"] is expected


def test_version_does_not_launch_or_remember_welcome(monkeypatch):
    def fail(**kwargs):
        pytest.fail("--version must not launch the TUI")

    monkeypatch.setattr("rvdlab.app.RvdLabApp", fail)
    assert main(["--root", str(ROOT), "--version", "--welcome"]) == 0
    assert welcome_needed()
