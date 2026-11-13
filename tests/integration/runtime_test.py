"""Subprocess checks: use simulation only; never discover or connect physical devices."""
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

rail, replay, root = map(Path, sys.argv[1:4])
root = root.resolve()
onnx_enabled = len(sys.argv) > 4 and sys.argv[4] == "ON"
scratch = Path(tempfile.mkdtemp(prefix="integration-", dir=Path.cwd())).resolve()
assert scratch.is_relative_to(Path.cwd().resolve())
model = root / "policies/IDSS_PFCR-DQP_s49_step10240000"
# The application baseline was recorded with the original checkpoint, which stays selectable.
legacy = root / "policies/E_PFCR-DQP_s41_step5120000"
legacy_config = scratch / "legacy-model.ini"
legacy_config.write_text("[controller]\nmodel_id = E_PFCR-DQP_s41_step5120000\n")
legacy_args = ["--config", legacy_config, "--model-dir", legacy]

def invoke(exe, args, rc):
    result = subprocess.run([str(exe), *map(str, args)], cwd=root, capture_output=True, text=True, timeout=30)
    assert result.returncode == rc, (result.returncode, result.stdout, result.stderr)
    return result

def run(name, args, rc, reason):
    directory = scratch / name
    result = invoke(rail, ["--fast", "--noise", "0", "--max-steps", "12", "--model-dir", model,
                          "--experiment-dir", directory, "--log", scratch / (name + ".csv"), *args], rc)
    summary = json.loads((directory / "summary.json").read_text())
    assert summary["reason"] == reason, (summary, result.stdout, result.stderr)
    assert summary["exit_code"] == rc
    assert json.loads((directory / "metadata.json").read_text())["hardware"] is False
    checksums = (directory / "checksums.sha256").read_text().splitlines()
    for line in checksums:
        digest, name = line.split("  ")
        assert hashlib.sha256((directory / name).read_bytes()).hexdigest() == digest, name
    assert any(line.endswith("  summary.json") for line in checksums)
    return directory

first = run("ideal", legacy_args, 1, "MAX_STEPS")
assert json.loads((first / "metadata.json").read_text())["model_id"] == legacy.name
baseline = json.loads((root / "tests/regression/application_baseline.json").read_text())
with (first / "trajectory.csv").open() as stream:
    actual = list(csv.DictReader(stream))
assert len(actual) == len(baseline["rows"])
for row, reference in zip(actual, baseline["rows"]):
    for column, expected in zip(baseline["columns"], reference):
        try:
            # Baseline CSV was printed to six decimals; cross-platform last-digit differences
            # are bounded by one printed unit. Strict core Golden tolerances remain unchanged.
            assert abs(float(row[column]) - float(expected)) <= 1.000001e-6, (column, row[column], expected)
        except ValueError:
            assert row[column] == expected, (column, row[column], expected)
if onnx_enabled:
    onnx_run = run("onnx", ["--onnx"], 1, "MAX_STEPS")
    invoke(replay, [onnx_run, model], 0)
    invoke(replay, [run("onnx-legacy", ["--onnx", *legacy_args], 1, "MAX_STEPS"), legacy], 0)
else:
    invoke(rail, ["--onnx"], 2)
second = run("ideal-repeat", legacy_args, 1, "MAX_STEPS")
assert (first / "controller.csv").read_bytes() == (second / "controller.csv").read_bytes()
assert (first / "trajectory.csv").read_bytes() == (second / "trajectory.csv").read_bytes()
assert "maximum output error 0" in invoke(replay, [first, legacy], 0).stdout
# The recorded model is found under policies/ when no model directory is given.
assert "maximum output error 0" in invoke(replay, [first], 0).stdout
invoke(replay, [first, model], 2)
# Default model: deterministic, replayable, and judged on the contact plane.
current = run("idss", [], 1, "MAX_STEPS")
assert json.loads((current / "metadata.json").read_text())["model_id"] == model.name
assert "maximum output error 0" in invoke(replay, [current], 0).stdout
contact = run("idss-contact", ["--max-steps", "-1"], 0, "CONTACT")
assert json.loads((contact / "summary.json").read_text())["outcome"] == "DOCKED"
assert any('"contact"' in line for line in (contact / "events.jsonl").read_text().splitlines())
run("frozen", ["--sim-freeze-at", "1"], 1, "SENSOR_FROZEN")
run("read-failed", ["--sim-fail-read-at", "3"], 1, "READ_FAILED")
run("initial-read-failed", ["--sim-fail-read-at", "0"], 3, "READ_FAILED")
run("send-failed", ["--sim-fail-send-at", "2"], 1, "SEND_FAILED")
run("stale", ["--sim-latency", "0.4"], 3, "STALE_MEASUREMENT")
bounded = run("bounded", ["--bounded", "--sim-latency", "0"], 1, "MAX_STEPS")
invoke(replay, [bounded, model], 0)
# PLC positioning plant: both command modes run; lead keeps a moving reference and stop points ahead.
run("plc-arrive", ["--plc"], 1, "MAX_STEPS")
trace = scratch / "plc-lead-trace.csv"
# The 5 mm bound was set on the original checkpoint's first 12 steps (1:1 x mapping, 0.2 m/s^2 plant).
lead = run("plc-lead", ["--plc", "--command-mode", "lead", "--sim-trace", trace, *legacy_args], 1, "MAX_STEPS")
invoke(replay, [lead, legacy], 0)
with (lead / "trajectory.csv").open() as stream:
    lead_rows = [r for r in csv.DictReader(stream) if r["status"] == "RUN"]
assert all(abs(float(r["track_err_m"])) < 0.005 for r in lead_rows), "lead mode lost the reference"
current_lead = run("plc-lead-idss", ["--plc", "--command-mode", "lead"], 1, "MAX_STEPS")
with (current_lead / "trajectory.csv").open() as stream:
    assert all(abs(float(r["track_err_m"])) < 0.02 for r in csv.DictReader(stream) if r["status"] == "RUN")
assert len(trace.read_text().splitlines()) > 1000
# Motion capture that keeps publishing a stale pose: the encoder exposes it, or else the moving reference does.
run("repeated-pose", ["--plc", "--sim-repeat-pose-at", "5"], 1, "MEASUREMENT_REPEATED")
config = scratch / "no-encoder.ini"
config.write_text("[simulation]\nencoder = false\n")
run("repeated-pose-lead", ["--plc", "--command-mode", "lead", "--sim-repeat-pose-at", "5", "--config", config], 1,
    "RAIL_TRACKING_ERROR")
# Track a deliberately unachievable motion profile: supervisor must stop rather than claim success.
config = scratch / "slow.ini"
config.write_text("[rail]\nacceleration_m_s2 = 0.001,0.001,0.001\n")
run("acceleration", ["--config", config], 1, "RAIL_ACCELERATION")
config.write_text("[simulation]\nbias_m = 3,0,0\n")
run("envelope", ["--config", config], 3, "MEASURED_RAIL_ENVELOPE")
for args in (["--lab"], ["--noise", "nan"], ["--noise", "wrong"], ["--noise", "-1"],
             ["--start", "21", "0", "0"], ["--max-steps", "1.5"], ["--config"], ["--seed", "-1"],
             ["--command-mode", "fast"], ["--sim-trace", scratch / "ideal-trace.csv"]):
    invoke(rail, args, 2)
config.write_text("[controller]\nperiod_s = 0.5\n")
invoke(rail, ["--config", config], 2)
# Existing experiment directories are never overwritten.
before = (first / "metadata.json").read_bytes()
invoke(rail, ["--fast", "--experiment-dir", first], 2)
assert before == (first / "metadata.json").read_bytes()
# A shortened or edited recording must fail integrity verification.
with (first / "controller.csv").open("a") as out:
    out.write("corrupted\n")
invoke(replay, [first, legacy], 2)
for directory in (model, legacy):
    manifest = json.loads((directory / "manifest.json").read_text())
    assert manifest["model_id"] == directory.name
    for filename, expected in manifest["artifacts"].items():
        assert hashlib.sha256((directory / filename).read_bytes()).hexdigest() == expected
print("runtime integration: deterministic logs, replay, bounded motion, fault injection, invalid CLI and integrity passed")
