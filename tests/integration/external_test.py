"""Exercise the external runtime against a loopback controller and simulated rail."""

import csv
import hashlib
import json
import math
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time


def run_case(rail, root, directory, replies, max_steps=2, disconnect=False):
    with socket.create_server(("127.0.0.1", 0)) as listener:
        listener.settimeout(8)
        command = [
            str(rail), "--config", str(root / "examples/simulink/rail-demo.ini"),
            "--external-port", str(listener.getsockname()[1]), "--external-name", "test-controller",
            "--max-steps", str(max_steps), "--experiment-dir", str(directory),
        ]
        process = subprocess.Popen(command, cwd=root, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        states = []
        result = None
        started = time.monotonic()
        try:
            peer, _ = listener.accept()
            with peer, peer.makefile("rw", encoding="ascii", newline="\n") as stream:
                peer.settimeout(5)
                for index, reply in enumerate(replies):
                    sample = stream.readline().split()
                    assert len(sample) == 7 and sample[0] == "STATE", sample
                    assert int(sample[1]) == index
                    assert math.isfinite(float(sample[2]))
                    assert int(sample[3]) >= 0
                    states.append([float(value) for value in sample[4:]])
                    if reply is not None:
                        stream.write(reply + "\n")
                        stream.flush()
                if not disconnect:
                    result = stream.readline().strip()
            output, errors = process.communicate(timeout=8)
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate(timeout=8)
        elapsed = time.monotonic() - started

    summary = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
    metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["controller"] == "external-reference" and metadata["hardware"] is False, metadata
    assert metadata["external_name"] == "test-controller" and "weights_sha256" not in metadata, metadata
    assert summary["exit_code"] == process.returncode, (summary, output, errors)
    assert summary["interventions"] is None, summary
    if result is not None and (result or summary["reason"] != "EXTERNAL_TIMEOUT"):
        assert result == f"END {summary['outcome']} {summary['reason']}", (result, summary, errors)
    for line in (directory / "checksums.sha256").read_text(encoding="utf-8").splitlines():
        digest, filename = line.split("  ", 1)
        assert hashlib.sha256((directory / filename).read_bytes()).hexdigest() == digest, filename
    return states, summary, elapsed


def assert_outcome(summary, outcome, reason):
    assert (summary["outcome"], summary["reason"]) == (outcome, reason), summary
    assert (summary["exit_code"] == 0) == (outcome != "FAULT"), summary


def main(rail, replay, root):
    with tempfile.TemporaryDirectory(prefix="external-runtime-") as temporary:
        scratch = Path(temporary)
        success = scratch / "complete"
        states, summary, _ = run_case(
            rail, root, success, ["REF 0 0.45 -0.15 0 -0.10 0 0", "REF 1 0.40 -0.15 0 -0.10 0 0"]
        )
        assert_outcome(summary, "COMPLETE", "MAX_STEPS")
        assert summary["controller_steps"] == 2, summary
        assert abs(states[0][0] - 0.5) < 1e-6 and abs(states[1][0] - 0.45) < 0.004, states
        with (success / "trajectory.csv").open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        assert rows[0]["status"] == "RUN" and abs(float(rows[0]["tx"]) - 0.005) < 1e-8, rows
        assert rows[-1]["status"] == "COMPLETE" and abs(float(rows[-1]["rail_x"]) - 0.01) < 0.001, rows
        replay_result = subprocess.run([str(replay), str(success)], cwd=root, capture_output=True, text=True, timeout=8)
        assert replay_result.returncode != 0 and "external" in replay_result.stderr.lower(), replay_result
        snapshot = subprocess.run([str(rail), "--config", str(success / "configuration.ini")],
                                  cwd=root, capture_output=True, text=True, timeout=8)
        assert snapshot.returncode == 2 and "--external-port" in snapshot.stderr, snapshot

        _, summary, _ = run_case(rail, root, scratch / "stop", ["STOP 0"])
        assert_outcome(summary, "STOPPED", "CLIENT_STOP")
        assert summary["controller_steps"] == 0
        _, summary, _ = run_case(rail, root, scratch / "protocol", ["REF 1 0.45 -0.15 0 -0.10 0 0"])
        assert_outcome(summary, "FAULT", "EXTERNAL_PROTOCOL")
        _, summary, _ = run_case(rail, root, scratch / "nonfinite", ["REF 0 0.45 -0.15 0 nan 0 0"])
        assert_outcome(summary, "FAULT", "EXTERNAL_PROTOCOL")
        _, summary, _ = run_case(rail, root, scratch / "speed", ["REF 0 0.45 -0.15 0 -1 0 0"])
        assert_outcome(summary, "FAULT", "RAIL_SPEED")
        _, summary, _ = run_case(rail, root, scratch / "acceleration", ["REF 0 0.45 -0.15 0 -0.30 0 0"])
        assert_outcome(summary, "FAULT", "RAIL_ACCELERATION")
        _, summary, _ = run_case(rail, root, scratch / "travel", ["REF 0 -0.5 -0.15 0 -0.10 0 0"])
        assert_outcome(summary, "FAULT", "RAIL_ENVELOPE")
        _, summary, _ = run_case(rail, root, scratch / "direction", ["REF 0 0.45 -0.15 0 0.10 0 0"])
        assert_outcome(summary, "FAULT", "RAIL_VELOCITY_DIRECTION")
        _, summary, _ = run_case(rail, root, scratch / "tracking", ["REF 0 0.1 -0.15 0 -0.01 0 0"])
        assert_outcome(summary, "FAULT", "RAIL_TRACKING_ERROR")
        _, summary, elapsed = run_case(rail, root, scratch / "timeout", [None])
        assert_outcome(summary, "FAULT", "EXTERNAL_TIMEOUT")
        assert elapsed < 4, elapsed
        _, summary, _ = run_case(rail, root, scratch / "disconnect", [], disconnect=True)
        assert_outcome(summary, "FAULT", "EXTERNAL_DISCONNECTED")
    print("external runtime: closed loop, records, deadlines, disconnect, motion checks and replay boundary passed")


if __name__ == "__main__":
    main(*(Path(arg).resolve() for arg in sys.argv[1:4]))
