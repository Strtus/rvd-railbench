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
model = root / "policies/E_PFCR-DQP_s41_step5120000"

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

first = run("ideal", [], 1, "MAX_STEPS")
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
else:
    invoke(rail, ["--onnx"], 2)
second = run("ideal-repeat", [], 1, "MAX_STEPS")
assert (first / "controller.csv").read_bytes() == (second / "controller.csv").read_bytes()
assert (first / "trajectory.csv").read_bytes() == (second / "trajectory.csv").read_bytes()
assert "maximum output error 0" in invoke(replay, [first, model], 0).stdout
run("frozen", ["--sim-freeze-at", "1"], 1, "SENSOR_FROZEN")
run("read-failed", ["--sim-fail-read-at", "3"], 1, "READ_FAILED")
run("initial-read-failed", ["--sim-fail-read-at", "0"], 3, "READ_FAILED")
run("send-failed", ["--sim-fail-send-at", "2"], 1, "SEND_FAILED")
run("stale", ["--sim-latency", "0.4"], 3, "STALE_MEASUREMENT")
bounded = run("bounded", ["--bounded", "--sim-latency", "0"], 1, "MAX_STEPS")
invoke(replay, [bounded, model], 0)
# Track a deliberately unachievable motion profile: supervisor must stop rather than claim success.
config = scratch / "slow.ini"
config.write_text("[rail]\nacceleration_m_s2 = 0.001,0.001,0.001\n")
run("acceleration", ["--config", config], 1, "RAIL_ACCELERATION")
config.write_text("[simulation]\nbias_m = 3,0,0\n")
run("envelope", ["--config", config], 3, "MEASURED_RAIL_ENVELOPE")
for args in (["--lab"], ["--noise", "nan"], ["--noise", "wrong"], ["--noise", "-1"],
             ["--start", "21", "0", "0"], ["--max-steps", "1.5"], ["--config"], ["--seed", "-1"]):
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
invoke(replay, [first, model], 2)
manifest = json.loads((model / "manifest.json").read_text())
for filename, expected in manifest["artifacts"].items():
    assert hashlib.sha256((model / filename).read_bytes()).hexdigest() == expected
print("runtime integration: deterministic logs, replay, bounded motion, fault injection, invalid CLI and integrity passed")
