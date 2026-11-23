import csv
import io

import pytest
from rich.console import Console
from textual.widgets import Sparkline, Static

from conftest import ROOT
from rvdlab.app import RvdLabApp
from rvdlab.plot import VelocitySparkline
from rvdlab.telemetry import EncoderSpeed


def sample(frame, stamp, position):
    return {"frame_id": str(frame), "t_encoder": str(stamp),
            **{f"enc_{a}": str(v) for a, v in zip("xyz", position)}}


def test_encoder_speed_uses_all_axes_and_actual_sample_interval():
    speed = EncoderSpeed()
    speed.extend([sample(10, 40, (1, 2, 3))])
    assert speed.value is None
    # 6 mm in X and 8 mm in Y over half a second: 20 mm/s.
    speed.extend([sample(13, 40.5, (1.006, 1.992, 3))])
    assert speed.value == pytest.approx(0.02)
    assert speed.components == pytest.approx((0.012, -0.016, 0.0))
    assert speed.interval_s == pytest.approx(0.5)
    # A new encoder sample at the same position is a measured zero, not missing data.
    speed.extend([sample(15, 42, (1.006, 1.992, 3))])
    assert speed.value == 0.0 and speed.interval_s == 1.5
    assert speed.components == (0.0, 0.0, 0.0)


def test_encoder_velocity_keeps_the_direction_on_each_axis():
    speed = EncoderSpeed()
    speed.extend([sample(1, 10, (1, 2, 3)), sample(2, 10.2, (0.996, 2.006, 2.992))])
    assert speed.components == pytest.approx((-0.02, 0.03, -0.04))


def test_encoder_speed_recovers_when_the_node_clock_advances():
    speed = EncoderSpeed()
    speed.extend([sample(1, 40, (0.002, 0, 0)), sample(2, 40, (0.002, 0, 0))])
    assert speed.components is None and speed.interval_s is None
    speed.extend([sample(3, 40.5, (0.008, 0, 0))])
    assert speed.components == pytest.approx((0.012, 0.0, 0.0))
    assert speed.interval_s == pytest.approx(0.5)


@pytest.mark.parametrize("bad", [
    {"t_encoder": ""}, {"t_encoder": "nan"}, {"t_encoder": "inf"},
    {"t_encoder": "40"}, {"t_encoder": "39"}, {"frame_id": "10"}, {"frame_id": "9"},
    {"enc_x": ""}, {"enc_y": "nan"}, {"enc_z": "inf"}, {"enc_x": "invalid"},
])
def test_encoder_speed_drops_invalid_samples_and_waits_for_a_new_pair(bad):
    speed = EncoderSpeed()
    speed.extend([sample(9, 39, (0, 0, 0)), sample(10, 40, (0.01, 0, 0))])
    assert speed.value == pytest.approx(0.01)
    speed.extend([{**sample(11, 41, (0.02, 0, 0)), **bad}])
    assert speed.value is None and speed.interval_s is None and speed.components is None
    speed.extend([sample(12, 42, (0.03, 0, 0))])
    assert speed.value is None
    speed.extend([sample(13, 43, (0.05, 0, 0))])
    assert speed.value == pytest.approx(0.02)


def test_encoder_speed_never_substitutes_model_or_command_velocity():
    speed = EncoderSpeed()
    rows = [{"frame_id": str(k), "t_encoder": str(k), "hvx": "3", "hvy": "4", "hvz": "0",
             "tvx": "1", "tvy": "2", "tvz": "3", "rail_x": str(k), "rail_y": "0", "rail_z": "0"}
            for k in range(2)]
    speed.extend(rows)
    assert speed.value is None


def status_text(screen):
    console = Console(width=100, file=io.StringIO(), record=True)
    console.print(screen.query_one("#run-status", Static).content)
    return console.export_text()


def append_samples(path, rows):
    new = not path.exists()
    with path.open("a", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        if new:
            writer.writeheader()
        writer.writerows(rows)


async def test_rendezvous_shows_raw_encoder_speed_and_labels_finished_samples(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles)
    async with app.run_test(size=(140, 55)) as pilot:
        await app.push_screen("rendezvous")
        await pilot.pause()
        screen = app.screen
        screen.bridge = object()  # Exercise the hardware display without a rail connection.
        directory = tmp_path / "recorded-run"
        directory.mkdir()
        screen.reset(directory)
        row = {"k": "0", "status": "RUN", "frame_id": "10", "hx": "20", "hy": "0", "hz": "0",
               "hvx": "-0.217", "hvy": "0.01", "hvz": "-0.007",
               # Deliberately disagree with the physical sensor log (e.g. a simulated axis).
               "enc_x": "100", "enc_y": "100", "enc_z": "100", "t_encoder": "0",
               "tvx": "0.04", "tvy": "0", "tvz": "0"}
        append_samples(directory / "sensors.csv", [sample(10, 40, (1, 2, 3))])
        screen.consume([row])
        assert "need 2 samples" in status_text(screen)
        append_samples(directory / "sensors.csv", [sample(13, 40.5, (1.006, 1.992, 3))])
        screen.consume([{**row, "k": "1", "frame_id": "13"}])
        text = status_text(screen)
        assert "Velocity [m/s]" in text and "X" in text and "Y" in text and "Z" in text
        assert "Hill model" in text and "-0.2170 +0.0100 -0.0070" in text
        assert "Rail encoder" in text and "+0.0120 -0.0160 +0.0000" in text
        assert "Encoder interval" in text and "0.500 s" in text
        assert "Hill velocity XYZ" in str(screen.query_one("#label-speed", Static).content)
        plot = screen.query_one("#spark-speed", VelocitySparkline)
        assert plot.samples[-1] == (-0.217, 0.01, -0.007)
        await pilot.pause()
        distance = screen.query_one("#spark-distance", Sparkline)
        assert plot.size.height == distance.size.height == 3
        for component in ("sparkline--min-color", "sparkline--max-color"):
            assert plot.get_component_styles(component).color == distance.get_component_styles(component).color
        console = Console(width=plot.size.width, file=io.StringIO(), record=True)
        console.print(plot.render())
        lines = console.export_text().splitlines()
        assert [line[0] for line in lines] == list("XYZ")
        assert all(len(line) == plot.size.width for line in lines)

        # A stopped/missing sensor log must not masquerade as a fresh speed.
        screen.consume([{**row, "k": "2", "frame_id": "14"}])
        assert "no fresh encoders" in status_text(screen)
        assert "+0.0120 -0.0160 +0.0000" not in status_text(screen)
        screen.stop_requested = True
        screen.finish(directory, 1)
        text = status_text(screen)
        assert "finished · last sample" in text and "+0.0120 -0.0160 +0.0000" in text

        # A new experiment cannot inherit the previous experiment's speed.
        screen.reset(tmp_path / "next-run")
        screen.consume([row])
        assert "waiting for encoders" in status_text(screen)
        assert "+0.0120 -0.0160 +0.0000" not in status_text(screen)
        screen.bridge = None


async def test_rendezvous_does_not_present_simulated_encoders_as_actual_speed(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles)
    async with app.run_test(size=(140, 55)) as pilot:
        await app.push_screen("rendezvous")
        await pilot.pause()
        screen = app.screen
        screen.reset(tmp_path / "simulated-run")
        screen.consume([{**sample(k, k, (k, 0, 0)), "k": str(k), "status": "RUN",
                         "hvx": "0.217", "hvy": "0", "hvz": "0"} for k in range(2)])
        text = status_text(screen)
        assert "Hill model" in text and "+0.2170 +0.0000 +0.0000" in text
        assert "Rail encoder" in text and "— (simulation)" in text
