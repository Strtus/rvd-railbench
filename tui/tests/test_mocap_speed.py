import csv
import io
import math

import pytest
from rich.console import Console
from textual.widgets import Static

from conftest import ROOT
from rvdlab.app import RvdLabApp
from rvdlab.telemetry import MocapSpeed


def pose_pair(frame, stamp, body=(2, 0, 0), reference=(1, 0, 0)):
    row = {"frame_id": str(frame)}
    for name, position in (("Target", body), ("Chaser", reference)):
        row.update({f"{name}_valid": "1", f"{name}_age_s": "0.01", f"{name}_t_source": str(stamp),
                    f"{name}_frames": str(frame),
                    **{f"{name}_p{a}": str(v) for a, v in zip("xyz", position)}})
    return row


def test_mocap_speed_uses_relative_xyz_and_camera_time():
    speed = MocapSpeed()
    first = pose_pair(10, 100, (5, 5, 5), (1, 2, 3))
    second = pose_pair(20, 100.5, (5.106, 4.808, 5.3), (1.1, 1.8, 3.3))
    # Common translation cancels. Relative motion is 6/8/0 mm over half a second.
    # Neither node timing nor per-body mapped rail coordinates define this speed.
    first.update(t_encoder="10", t_node="10.1", Target_rail_x="0", Chaser_rail_x="0")
    second.update(t_encoder="14", t_node="14.6", Target_rail_x="50", Chaser_rail_x="0")
    speed.extend([first, second])
    assert speed.value == pytest.approx(0.02)
    assert speed.components == pytest.approx((0.012, 0.016, 0.0))
    assert speed.interval_s == 0.5


def test_mocap_relative_speed_includes_lateral_motion_at_constant_range():
    speed = MocapSpeed()
    speed.extend([pose_pair(1, 10, (1, 0, 0), (0, 0, 0)), pose_pair(2, 12, (0, 1, 0), (0, 0, 0))])
    assert speed.value == pytest.approx(math.sqrt(2) / 2)
    assert speed.components == pytest.approx((-0.5, 0.5, 0.0))


def test_mocap_relative_velocity_keeps_xyz_signs_in_the_raw_camera_frame():
    speed = MocapSpeed()
    speed.extend([pose_pair(1, 100), pose_pair(2, 100.5, (2.09, -0.02, 0.03), (1.1, 0, 0))])
    assert speed.components == pytest.approx((-0.02, -0.04, 0.06))


def test_stationary_relative_pose_with_new_camera_samples_is_measured_zero():
    speed = MocapSpeed()
    speed.extend([pose_pair(1, 10), pose_pair(2, 11)])
    assert speed.value == 0.0 and speed.interval_s == 1
    assert speed.components == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("bad", [
    {"Target_valid": "0"}, {"Chaser_valid": "0"}, {"Target_valid": ""},
    {"Target_t_source": ""}, {"Chaser_t_source": "nan"}, {"Target_t_source": "inf"},
    {"Target_t_source": "100.52"},
    {"Target_t_source": "100", "Chaser_t_source": "100"},
    {"Target_t_source": "99", "Chaser_t_source": "99"},
    {"Target_frames": "10"}, {"Chaser_frames": "9"}, {"Target_frames": "nan"},
    {"Target_px": ""}, {"Chaser_py": "inf"}, {"Target_pz": "invalid"},
    {"Chaser_age_s": "-0.1"}, {"Target_age_s": ""},
])
def test_invalid_or_repeated_mocap_requires_two_new_valid_pairs(bad):
    speed = MocapSpeed()
    speed.extend([pose_pair(9, 99, (1.98, 0, 0)), pose_pair(10, 100)])
    assert speed.value == pytest.approx(0.02)
    speed.extend([{**pose_pair(11, 100.5, (2.01, 0, 0)), **bad}])
    assert speed.value is None and speed.interval_s is None and speed.components is None
    speed.extend([pose_pair(12, 101, (2.02, 0, 0))])
    assert speed.value is None
    speed.extend([pose_pair(13, 101.5, (2.03, 0, 0))])
    assert speed.value == pytest.approx(0.02)


def test_mocap_without_both_trackers_never_falls_back_to_encoders():
    speed = MocapSpeed()
    pair = pose_pair(10, 100)
    row = {k: v for k, v in pair.items() if not k.startswith("Chaser_")}
    row.update(t_encoder="100", enc_x="1", enc_y="0", enc_z="0", hvx="0.217")
    speed.extend([row, {**row, "enc_x": "2", "t_encoder": "101"}])
    assert speed.value is None and speed.reason == "no mocap pair"


def status_text(screen):
    console = Console(width=100, file=io.StringIO(), record=True)
    console.print(screen.query_one("#run-status", Static).content)
    return console.export_text()


def append_sample(path, row):
    new = not path.exists()
    with path.open("a", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(row))
        if new:
            writer.writeheader()
        writer.writerow(row)


async def test_rendezvous_displays_three_speeds_and_drops_lost_mocap(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles)
    async with app.run_test(size=(140, 55)) as pilot:
        await app.push_screen("rendezvous")
        await pilot.pause()
        screen = app.screen
        screen.bridge = object()  # Display recorded samples without a hardware connection.
        directory = tmp_path / "recorded-run"
        directory.mkdir()
        screen.reset(directory)
        trajectory = {"k": "0", "status": "RUN", "frame_id": "10", "hx": "20", "hy": "0", "hz": "0",
                      "hvx": "0.217", "hvy": "0", "hvz": "0"}
        first = {**pose_pair(10, 100), "t_encoder": "40", "enc_x": "1", "enc_y": "2", "enc_z": "3"}
        second = {**pose_pair(11, 100.5, (2.115, 0, 0), (1.1, 0, 0)),
                  "t_encoder": "40.5", "enc_x": "1.006", "enc_y": "1.992", "enc_z": "3"}
        append_sample(directory / "sensors.csv", first)
        screen.consume([trajectory])
        assert "need 2 pose pairs" in status_text(screen)
        append_sample(directory / "sensors.csv", second)
        screen.consume([{**trajectory, "k": "1", "frame_id": "11"}])
        text = status_text(screen)
        assert "Hill model" in text and "+0.2170 +0.0000 +0.0000" in text
        assert "Rail encoder" in text and "+0.0120 -0.0160 +0.0000" in text
        assert "Mocap raw rel." in text and "+0.0300 +0.0000 +0.0000" in text
        assert "Mocap interval" in text and "0.500 s" in text

        screen.consume([{**trajectory, "k": "2", "frame_id": "12"}])
        assert "no fresh mocap" in status_text(screen)
        assert "+0.0300 +0.0000 +0.0000" not in status_text(screen)
        append_sample(directory / "sensors.csv", {**second, "frame_id": "12", "Target_valid": "0"})
        screen.consume([{**trajectory, "k": "2", "frame_id": "12"}])
        assert "Target invalid" in status_text(screen)
        assert "+0.0300 +0.0000 +0.0000" not in status_text(screen)
        assert app.rail_control_state == "running"  # The mocap display does not drive control.

        for frame, stamp, position in ((13, 101, (2.13, 0, 0)), (14, 101.5, (2.145, 0, 0))):
            append_sample(directory / "sensors.csv", {**second, **pose_pair(frame, stamp, position, (1.1, 0, 0))})
            screen.consume([{**trajectory, "k": str(frame - 10), "frame_id": str(frame)}])
        assert screen.mocap_speed.value == pytest.approx(0.03)
        screen.stop_requested = True
        screen.finish(directory, 1)
        assert "finished · last sample" in status_text(screen)
        assert "+0.0300 +0.0000 +0.0000" in status_text(screen)
        screen.reset(tmp_path / "next-run")
        screen.consume([trajectory])
        assert "waiting for mocap" in status_text(screen)
        assert "+0.0300 +0.0000 +0.0000" not in status_text(screen)
        screen.bridge = None


async def test_simulation_does_not_claim_a_camera_observation(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles)
    async with app.run_test(size=(140, 55)) as pilot:
        await app.push_screen("rendezvous")
        await pilot.pause()
        screen = app.screen
        screen.reset(tmp_path / "simulation")
        screen.consume([{"k": "0", "status": "RUN", "hvx": "0.2", "hvy": "0", "hvz": "0"}])
        text = status_text(screen)
        assert "Mocap raw rel." in text and "— (simulation)" in text
