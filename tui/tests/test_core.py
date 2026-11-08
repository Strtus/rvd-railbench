import math
from dataclasses import replace
from pathlib import Path

import pytest

from rvdlab.config import (ConfigError, Profile, RailConfig, apply_file, envelope_violations, hill_to_rail,
                           load_profile, rail_to_hill, simulation_override)
from rvdlab.experiments import new_experiment_dir
from rvdlab.profiles import RailProfile, check, load_rail_profile, read_active, save, set_active
from rvdlab.project import config_files, load_project, system_status
from rvdlab.sweep import RunResult, evaluate, fit_mapping, validation_starts
from rvdlab.telemetry import CsvTail, Trajectory, frozen_run, stall_run, summarize

from conftest import ROOT

# A compact rail: 8 m docking axis approached towards +X, small lateral travel, low speed limits.
COMPACT = RailConfig(minimum_m=(0.0, -0.1, 0.0), maximum_m=(8.0, 0.4, 0.6),
                     velocity_m_s=(0.05, 0.015, 0.025), acceleration_m_s2=(0.01, 0.01, 0.01),
                     sign=(-1.0, 1.0, 1.0), scale=(0.08, 0.08, 0.1), target_m=(8.0, 0.0, 0.35))


def test_checked_in_profile_matches_compiled_defaults():
    profile = load_profile(ROOT)
    assert [p.name for p in profile.files] == ["rail.ini", "pfcr_dqp.ini", "simulation.ini"]
    assert profile.rail == RailConfig()
    assert profile.simulation.start_hill_m == (18.0, 0.3, -0.2)


def test_rejects_what_rail_main_rejects(tmp_path):
    bad = tmp_path / "bad.ini"
    for text in ("[rail]\nunknown = 1\n", "[rail]\nminimum_m = 1,2\n", "[mapping]\nsign = 1,1,1\nsign = 1,1,1\n",
                 "[controller]\nmodel_id = other\n", "[nowhere]\n"):
        bad.write_text(text, encoding="utf-8")
        with pytest.raises(ConfigError):
            apply_file(Profile(), bad)


def test_override_file_round_trips(tmp_path):
    path = tmp_path / "override.ini"
    path.write_text(simulation_override((100.0, -0.4, 0.1), "bounded", True, 12, 7, fail_send_at=5), encoding="utf-8")
    profile = Profile()
    apply_file(profile, path)
    s = profile.simulation
    assert (s.start_hill_m, s.mode, s.fast, s.max_steps, s.seed) == ((100.0, -0.4, 0.1), "bounded", True, 12, 7)
    assert (s.freeze_at, s.fail_read_at, s.fail_send_at) == (-1, -1, 5)


def test_command_mode_and_plc_plant_keys(tmp_path):
    path = tmp_path / "lead.ini"
    path.write_text("[rail]\ncommand_mode = lead\nlead_s = 0.4\ntracking_gain_per_s = 0.8\nsensor_disagreement_m = 0.02\n"
                    "[simulation]\nmode = plc\nplc_acceleration_m_s2 = 0.1,0.2,0.3\nplc_delay_s = 0.05\n"
                    "repeat_pose_at = 3\nencoder = false\n", encoding="utf-8")
    profile = Profile()
    apply_file(profile, path)
    r, s = profile.rail, profile.simulation
    assert (r.command_mode, r.lead_s, r.tracking_gain_per_s, r.sensor_disagreement_m) == ("lead", 0.4, 0.8, 0.02)
    assert (s.mode, s.plc_acceleration_m_s2, s.plc_delay_s, s.repeat_pose_at, s.encoder) == (
        "plc", (0.1, 0.2, 0.3), 0.05, 3, False)
    lead = RailProfile("lead", replace(COMPACT, command_mode="lead", lead_s=0.4), (99.9, 0.0, -0.5))
    assert load_rail_profile(save(tmp_path, lead)) == lead
    for text in ("[rail]\ncommand_mode = fast\n", "[simulation]\nmode = turbo\n"):
        path.write_text(text, encoding="utf-8")
        with pytest.raises(ConfigError):
            apply_file(Profile(), path)
    with pytest.raises(ConfigError):
        check(RailProfile("gain", replace(COMPACT, tracking_gain_per_s=1.5), (99.9, 0.0, -0.5)))


def test_rail_profile_round_trip_and_active_pointer(tmp_path):
    profile = RailProfile("lab.rail-2", COMPACT, (99.9, 0.0, -0.5))
    path = save(tmp_path, profile, ("calibrated 2026-11-06",))
    assert path.read_text(encoding="utf-8").splitlines()[1] == "# calibrated 2026-11-06"
    assert load_rail_profile(path) == profile
    assert read_active(tmp_path) is None
    set_active(tmp_path, "lab.rail-2")
    assert read_active(tmp_path) == "lab.rail-2"
    files, name = config_files(ROOT, tmp_path, None)
    assert name == "lab.rail-2" and files[-1] == path and [f.name for f in files[:2]] == ["pfcr_dqp.ini", "simulation.ini"]


def test_profile_rejects_bad_names_and_mappings(tmp_path):
    with pytest.raises(ConfigError):
        save(tmp_path, RailProfile("../escape", COMPACT, (50.0, 0.0, 0.0)))
    with pytest.raises(ConfigError):
        save(tmp_path, RailProfile("ok", replace(COMPACT, axis=(0, 0, 2)), (50.0, 0.0, 0.0)))


def test_explicit_configs_bypass_profiles(tmp_path, profiles):
    files, name = config_files(ROOT, profiles, [ROOT / "configs" / "rail.ini"])
    assert name is None and files == [ROOT / "configs" / "rail.ini"]


def test_frame_map_round_trip_with_permutation_and_sign():
    rail = RailConfig(axis=(1, 0, 2), sign=(-1.0, 1.0, 1.0), scale=(0.1, 0.09, 0.1), target_m=(0.2, 10.0, 0.37))
    hill = (100.0, 0.4, -0.93)
    position = hill_to_rail(rail, hill)
    assert position[1] == pytest.approx(0.0)
    assert all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(rail_to_hill(rail, position), hill))


def test_envelope_violation_reports_distance():
    assert envelope_violations(RailConfig(), (10.0, 0.0, 0.0)) == []
    assert envelope_violations(RailConfig(), (-0.002, 0.0, 0.6)) == ["X below minimum by 2.0 mm",
                                                                      "Z above maximum by 100.0 mm"]


def reference_run(points, speeds):
    rows = [{"k": str(k), "status": "RUN", "hx": str(x), "hy": str(y), "hz": str(z),
             "tvx": str(vx), "tvy": str(vy), "tvz": str(vz)} for k, ((x, y, z), (vx, vy, vz)) in enumerate(zip(points, speeds))]
    return RunResult(points[0], "DOCKED", "CAPTURED", len(rows), rows)


def test_fit_uses_the_tighter_of_travel_and_speed():
    run = reference_run([(100.0, -1.0, 2.0), (50.0, 4.0, -2.0), (0.8, 0.0, 0.0)],
                        [(0.3, 0.1, 0.25), (0.1, 0.05, 0.1), (0.0, 0.0, 0.0)])
    fit = fit_mapping(COMPACT, [run], margin=0.1)
    # x: travel 8*0.9/100 = 0.072 < speed 0.05*0.9/0.3 = 0.15
    assert fit.scale[0] == pytest.approx(0.072) and fit.limited_by[0] == "travel"
    # y: travel 0.5*0.9/5 = 0.09 < speed 0.015*0.9/0.1 = 0.135
    assert fit.scale[1] == pytest.approx(0.09) and fit.limited_by[1] == "travel"
    # z: speed 0.025*0.9/0.25 = 0.09 < travel 0.6*0.9/4 = 0.135
    assert fit.scale[2] == pytest.approx(0.09) and fit.limited_by[2] == "speed"
    fitted = replace(COMPACT, scale=fit.scale, target_m=fit.target)
    for point in ((100.0, -1.0, 2.0), (50.0, 4.0, -2.0), (0.0, 0.0, 0.0)):
        assert not envelope_violations(fitted, hill_to_rail(fitted, point))


def test_validation_starts_pull_x_inside_the_travel():
    starts = validation_starts(COMPACT)
    assert starts and all(s[0] == pytest.approx(99.9) for s in starts)
    assert all(not envelope_violations(COMPACT, hill_to_rail(COMPACT, s)) for s in starts)
    assert {s[1] for s in starts} <= {-0.4, 0.0, 0.4}


def test_evaluate_reports_speed_use_and_clearance():
    rows = [{"k": "0", "status": "RUN", "tvx": "0.02", "tvy": "0.006", "tvz": "-0.01", "tx": "1.0", "ty": "0.35", "tz": "0.3"}]
    check = evaluate(COMPACT, RunResult((99.9, 0.0, 0.0), "DOCKED", "CAPTURED", 1, rows))
    assert check.speed_use == pytest.approx((0.4, 0.4, 0.4))
    assert check.clearance_m == pytest.approx(0.05)


def row(k, rail, target, status="RUN"):
    return {"k": str(k), "status": status, "rail_x": str(rail[0]), "rail_y": str(rail[1]), "rail_z": str(rail[2]),
            "tx": str(target[0]), "ty": str(target[1]), "tz": str(target[2])}


def test_stall_run_finds_a_rail_that_stops_moving():
    rows, x = [], 0.0
    for k in range(10):
        moving = k < 4 or k >= 8
        rows.append(row(k, (x, 0, 0), (x + 0.027, 0, 0)))
        x += 0.027 if moving else 0.0
    assert stall_run(rows) == (4, 5)


def test_frozen_run_finds_a_repeated_pose():
    rows = [row(0, (3.29, 0.36, 0.33), (3.31, 0.36, 0.33))]
    rows += [row(k, (3.297878, 0.362962, 0.337), (3.32556, 0.36, 0.33)) for k in range(1, 6)]
    rows.append(row(6, (3.297878, 0.362962, 0.337001), (3.32556, 0.36, 0.33)))
    assert frozen_run(rows) == (4, 2)


def test_stall_run_ignores_steps_without_commanded_motion():
    assert stall_run([row(k, (0, 0, 0), (0.001, 0, 0)) for k in range(5)]) == (0, -1)


def test_summary_reports_speed_use_and_envelope_clearance():
    rows = []
    for k in range(3):
        r = row(k, (0.1 * k, 0, 0), (0.1 * (k + 1), 0, 0))
        r.update(tvx="0.25", tvy="0", tvz="0", hx=str(18 - k), hy="0", hz="0", hvx="0.1", hvy="0", hvz="0")
        rows.append(r)
    metrics = {m.name: m for m in summarize(Trajectory(rows), RailConfig())}
    assert metrics["X speed"].note == "50% of 0.5 m/s"
    assert metrics["X target range"].note == "clearance 100 mm"
    assert metrics["Stalled steps"].value == "0"


def test_csv_tail_returns_complete_rows_only(tmp_path):
    path = tmp_path / "trajectory.csv"
    path.write_bytes(b"k,status\n0,RUN\n1,R")
    tail = CsvTail(path)
    assert tail.poll() == [{"k": "0", "status": "RUN"}]
    with path.open("ab") as stream:
        stream.write(b"UN\n")
    assert tail.poll() == [{"k": "1", "status": "RUN"}]


def test_experiment_directory_name_matches_rail_main_format(tmp_path):
    name = new_experiment_dir(tmp_path).name
    assert len(name) == len("20261107T153341-954074Z") and name[8] == "T" and name.endswith("Z")


def test_status_never_claims_a_device_connection(tmp_path):
    statuses = system_status(load_project(ROOT, profiles=tmp_path))
    assert statuses[0].label == "Rail Profile" and not statuses[0].ok
    devices = {s.label: s for s in statuses[3:]}
    assert set(devices) == {"Motion Controller", "Motion Capture", "LiDAR / Camera"}
    assert not any(s.ok for s in devices.values())
