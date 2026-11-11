import hashlib
import json
import math
from pathlib import Path

import pytest
from textual.widgets import Input

from conftest import ROOT
from rvdlab.__main__ import main
from rvdlab.app import RvdLabApp
from rvdlab.screens.welcome import OrbitalArt, WelcomeScreen
from rvdlab.welcome import (CURSOR_DURATION, DURATION, ECCENTRICITY, FPS, LETTER_TIMES,
                           ORBIT_DURATION, REVEAL_DURATION, REVEAL_FRAMES, TYPE_START, OrbitalAnimation,
                           WelcomeAnimation, _projected_geometry, eccentric_anomaly,
                           mark_welcome_seen, welcome_needed)


@pytest.fixture(autouse=True)
def welcome_home(tmp_path, monkeypatch):
    directory = tmp_path / "user-data"
    monkeypatch.setenv("RVDLAB_HOME", str(directory))
    return directory


def test_complete_outline_preserves_recorded_powershell_geometry():
    reference = json.loads((Path(__file__).parent / "fixtures" / "welcome_frames.json").read_text(encoding="utf-8"))
    animation = OrbitalAnimation()
    frame = next(frame for frame in reference["frames"] if frame["index"] == 90)
    text = animation.render(REVEAL_FRAMES)
    assert hashlib.sha256(text.plain.encode("utf-8")).hexdigest() == frame["text_sha256"]


def test_reveal_then_two_and_a_half_seconds_before_shrink():
    assert REVEAL_DURATION == 3.0
    assert REVEAL_FRAMES == 3 * FPS
    assert ORBIT_DURATION == 2.5
    assert DURATION == 5.5
    scene = WelcomeAnimation(120, 40)
    complete = scene.render(REVEAL_DURATION).plain
    assert scene.render(DURATION - 1 / FPS).plain == complete
    assert scene.render(DURATION).plain == complete
    assert scene.render(DURATION + .8).plain != complete


def test_kepler_motion_sweeps_equal_areas_and_speeds_up_near_focus():
    def points(start):
        angles = [eccentric_anomaly(start + .2 * i / 1000) for i in range(1001)]
        return [(math.cos(angle) - ECCENTRICITY, .545 * math.sin(angle)) for angle in angles]

    near, far = points(0), points(math.pi)

    def area(points):
        return sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(points, points[1:])) / 2

    assert area(near) == pytest.approx(area(far), rel=1e-5)
    near_speed = math.dist(near[0], near[1])
    far_speed = math.dist(far[0], far[1])
    assert near_speed > 5 * far_speed


@pytest.mark.parametrize("size", [(108, 27), (76, 18), (40, 10)])
def test_comet_does_not_light_cells_shared_with_other_orbits(size):
    animation = OrbitalAnimation(*size)
    assert any(owner.bit_count() > 1 for owner in animation.owners)
    lit = set()
    for frame in range(REVEAL_FRAMES, REVEAL_FRAMES + 360):
        masks, hot = animation.cells(frame)
        assert masks == animation.masks  # shared outlines must not disappear
        for index in hot:
            assert animation.owners[index].bit_count() == 1
            assert animation.owners[index] != 1 << 6  # central hub stays dim
        lit.update(hot)
    assert lit


@pytest.mark.parametrize("size", [(120, 40), (80, 24), (40, 12)])
def test_compact_logo_keeps_shared_cells_dim_through_whole_cycle(size):
    scene = WelcomeAnimation(*size)
    _, owners = _projected_geometry(*size, scene.target_scale, scene.target_cx, scene.target_cy)
    lit = set()
    for step in range(360):
        text = scene.render(TYPE_START + CURSOR_DURATION + step / FPS)
        for span in text.spans:
            row, col = divmod(span.start, size[0] + 1)
            if col >= scene.title_x:
                continue  # white wordmark
            index = row * size[0] + col
            assert owners[index].bit_count() == 1
            assert owners[index] != 1 << 6
            lit.add(index)
    assert lit


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


async def test_transform_keeps_playing_until_key_and_survives_resize(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles, welcome=True)
    async with app.run_test(size=(120, 40)) as pilot:
        screen = app.screen
        assert isinstance(screen, WelcomeScreen)
        assert screen.query_one(OrbitalArt).size == (120, 40)
        assert not screen.query("#welcome-title, #welcome-hint")
        await pilot.resize_terminal(80, 24)
        await pilot.pause()
        assert screen.query_one(OrbitalArt).size == (80, 24)
        screen._started -= TYPE_START + 15
        screen._tick()
        await pilot.pause()
        assert isinstance(app.screen, WelcomeScreen)
        assert welcome_needed()
        await pilot.press("enter")
        await pilot.pause()
        assert type(app.screen).__name__ == "HomeScreen"
        assert not welcome_needed()
        # A queued tick after dismissal must not touch the new screen.
        screen._tick()


def test_opening_has_only_braille_and_no_captions():
    scene = WelcomeAnimation(120, 40)
    for time in (0, 1.5, REVEAL_DURATION, DURATION - .01, TYPE_START - .01):
        text = scene.render(time).plain
        assert all(c in " \n" or 0x2800 <= ord(c) <= 0x28ff for c in text)


@pytest.mark.parametrize("size", [(120, 40), (80, 24), (40, 12), (25, 8)])
def test_final_logo_and_wordmark_have_identical_occupied_rows(size):
    scene = WelcomeAnimation(*size)
    lines = scene.render(TYPE_START + CURSOR_DURATION + 1).plain.splitlines()
    logo_rows = [r for r, line in enumerate(lines) if line[:scene.title_x].strip()]
    text_rows = [r for r, line in enumerate(lines) if line[scene.title_x:].strip()]
    assert logo_rows == text_rows == list(range(scene.logo_y, scene.logo_y + scene.logo_height))
    assert len(lines) == size[1] and all(len(line) == size[0] for line in lines)
    assert [lines[scene.logo_y+r][scene.title_x:scene.title_x+len(line)]
            for r, line in enumerate(scene.wordmark)] == scene.wordmark


def test_zoom_has_intermediate_sizes_and_no_wordmark_yet():
    scene = WelcomeAnimation(120, 40)
    heights = []
    for time in (DURATION, DURATION+.4, DURATION+.8, DURATION+1.2, TYPE_START-.001):
        occupied = [i for i, line in enumerate(scene.render(time).plain.splitlines()) if line.strip()]
        heights.append(occupied[-1] - occupied[0] + 1)
    assert heights == sorted(heights, reverse=True)
    assert len(set(heights)) == 5
    assert heights[-1] == 6


def test_cursor_blinks_before_typing_and_letters_follow_site_cadence():
    scene = WelcomeAnimation(120, 40)

    def title(time):
        lines = scene.render(TYPE_START + time).plain.splitlines()
        return [line[scene.title_x:] for line in lines[scene.logo_y:scene.logo_y+scene.logo_height]]

    assert all(line.startswith("█") for line in title(.05))
    assert all(not line.strip() for line in title(.35))
    for letters, time in enumerate(LETTER_TIMES, start=1):
        visible = scene.stops[letters-1]
        assert [line[:visible] for line in title(time+.001)] == [line[:visible].ljust(visible) for line in scene.wordmark]
    assert all(not line[scene.wordmark_width:].strip() for line in title(CURSOR_DURATION+.1))


def test_orbits_continue_after_wordmark_and_multiple_cycles():
    scene = WelcomeAnimation(120, 40)
    first = scene.render(TYPE_START + 10)
    second = scene.render(TYPE_START + 10.5)
    later = scene.render(TYPE_START + 120)
    assert first.plain == second.plain == later.plain  # same geometry and complete text
    assert first.spans != second.spans and later.spans != second.spans  # comet keeps moving
    assert OrbitalAnimation().cells(10000)[1]


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
