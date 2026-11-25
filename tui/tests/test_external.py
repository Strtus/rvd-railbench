"""Run the external control gateway against a loopback-only rail-node simulator."""

import csv
import json
import socket
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from pathlib import Path

import pytest

from conftest import ROOT
from rvdlab.project import find_build

pytest.importorskip("rvd_railnode")
from rvd_railnode.client import NodeClient, NodeError  # noqa: E402
from rvd_railnode.node import Node, Settings  # noqa: E402
from rvd_railnode.profile import SIMULATOR, validate  # noqa: E402
from rvd_railnode.server import NodeServer, UserStore  # noqa: E402
from rvd_railnode.sim import SimBackend  # noqa: E402

requires_runtime = pytest.mark.skipif(find_build(ROOT).rail_main is None, reason="rail_main is not built")

LIMITS = """[rail]
minimum_m = 0,-0.25,-0.25
maximum_m = 2,0.25,0.25
velocity_m_s = 0.1,0.1,0.1
acceleration_m_s2 = 0.1,0.1,0.1
"""
MAPPING = """[mapping]
axis = 0,1,2
sign = -1,1,1
scale = 0.1,0.2,0.3
target_m = 1.1,0,0
"""


@pytest.fixture
def simulated_node(tmp_path):
    profile = {**SIMULATOR, "axes": {**SIMULATOR["axes"], "x": {**SIMULATOR["axes"]["x"], "start": 1.0}}}
    rail = Node(SimBackend(validate(profile), True), Settings(poll_s=0.02))
    token = "a" * 40
    users = tmp_path / "users.json"
    users.write_text(json.dumps({"alice": token}), encoding="utf-8")
    with NodeServer(("127.0.0.1", 0), rail, UserStore(users)) as server:
        connection = tmp_path / "connection.json"
        connection.write_text(json.dumps({"url": f"http://127.0.0.1:{server.server_address[1]}", "token": token}),
                              encoding="utf-8")
        rail.start()
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        thread.start()
        try:
            yield connection, rail
        finally:
            server.shutdown()
            thread.join(timeout=5)
            rail.close()


@contextmanager
def external_process(tmp_path, connection, config_text):
    config = tmp_path / "rail.ini"
    config.write_text(config_text, encoding="utf-8")
    ready = tmp_path / "ready.json"
    command = [
        sys.executable, "-m", "rvdlab", "external", "--root", str(ROOT), "--config", str(config),
        "--profiles", str(tmp_path / "profiles"), "--experiments", str(tmp_path / "experiments"),
        "--ready-file", str(ready), "--max-steps", "4", "--node", str(connection),
    ]
    with subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) as process:
        try:
            yield process, ready
        except BaseException:
            try:
                _, stderr = process.communicate(timeout=2)
            except subprocess.TimeoutExpired:
                pass
            else:
                print(stderr, file=sys.stderr)
            raise
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)


def ready_endpoint(process, ready):
    deadline = time.monotonic() + 10
    while not ready.is_file():
        if process.poll() is not None:
            _, stderr = process.communicate()
            pytest.fail(f"The gateway exited before publishing its endpoint: {stderr}")
        assert time.monotonic() < deadline, "The gateway did not publish its endpoint"
        time.sleep(0.01)
    endpoint = json.loads(ready.read_text(encoding="utf-8"))
    assert endpoint["host"] == "127.0.0.1" and endpoint["period_s"] == 1.0
    assert isinstance(endpoint["token"], str) and endpoint["token"]
    return endpoint


def authenticate(stream, endpoint):
    stream.write(f"HELLO {endpoint['token']}\n".encode("ascii"))


def read_state(stream, step):
    fields = stream.readline().decode("ascii").split()
    assert len(fields) == 7 and fields[:2] == ["STATE", str(step)], fields
    return int(fields[3]), [float(value) for value in fields[4:]]


def finished_run(process, ready, endpoint, status=None):
    _, stderr = process.communicate(timeout=10)
    if status is not None:
        assert process.returncode == status, stderr
    assert not ready.exists()
    directory = Path(endpoint["experiment_dir"])
    summary = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
    assert summary["exit_code"] == process.returncode, (summary, stderr)
    return directory, summary


@requires_runtime
@pytest.mark.parametrize("config", [LIMITS, LIMITS.replace("acceleration_m_s2 = 0.1,0.1,0.1\n", "") + MAPPING])
def test_node_requires_explicit_limits_and_mapping(tmp_path, config):
    # An intentionally unusable endpoint must never be reached.
    connection = tmp_path / "unused.json"
    connection.write_text("{}", encoding="utf-8")
    with external_process(tmp_path, connection, config) as (process, ready):
        stdout, stderr = process.communicate(timeout=10)
        assert process.returncode == 2, (stdout, stderr)
        assert "mapping" in stderr.lower() or "acceleration" in stderr.lower(), stderr
        assert not ready.exists()


@requires_runtime
def test_external_moves_and_releases_a_simulated_node(tmp_path, simulated_node):
    connection, rail = simulated_node
    with external_process(tmp_path, connection, LIMITS + MAPPING) as (process, ready):
        endpoint = ready_endpoint(process, ready)
        assert rail.snapshot()["control"] is None
        with socket.create_connection((endpoint["host"], endpoint["port"]), timeout=10) as rejected:
            rejected.sendall(b"HELLO invalid-token\n")
            assert rejected.recv(1) == b""
        assert rail.snapshot()["control"] is None
        with socket.create_connection((endpoint["host"], endpoint["port"]), timeout=10) as peer:
            with peer.makefile("rwb", buffering=0) as stream:
                authenticate(stream, endpoint)
                frame, initial = read_state(stream, step=0)
                assert initial == pytest.approx([1.0, 0.0, 0.0])
                assert rail.snapshot()["control"] is not None
                stream.write(b"REF 0 0.8 0 0 -0.25 0 0\n")
                next_frame, measured = read_state(stream, step=1)
                state = rail.snapshot()
                assert state["target"]["x"] == pytest.approx(1.02)
                assert state["speed"]["x"] == pytest.approx(0.025)  # Below the legacy bridge's 0.05 m/s floor.
                assert next_frame > frame and measured == pytest.approx([0.8, 0.0, 0.0], abs=0.01)
                stream.write(b"REF 1 0.6 0 0 -0.25 0 0\n")
                final_frame, final = read_state(stream, step=2)
                assert final_frame > next_frame and final == pytest.approx([0.6, 0.0, 0.0], abs=0.01)
                stream.write(b"STOP 2\n")
                assert stream.readline() == b"END STOPPED CLIENT_STOP\n"

        directory, summary = finished_run(process, ready, endpoint, status=0)
        assert summary["outcome"] == "STOPPED" and summary["reason"] == "CLIENT_STOP"
        assert rail.snapshot()["control"] is None
        metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
        assert metadata["hardware"] is True
        with (directory / "trajectory.csv").open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        assert rows[0]["status"] == "RUN"
        assert float(rows[0]["hx"]) == pytest.approx(1.0)
        assert float(rows[0]["nx"]) == pytest.approx(0.8)
        assert float(rows[0]["tx"]) == pytest.approx(1.02)
        assert float(rows[0]["tvx"]) == pytest.approx(0.025)


@requires_runtime
@pytest.mark.parametrize("reference", [b"REF 0 -10 0 0 -0.25 0.001 0.001\n", None],
                         ids=["rail-envelope", "disconnect"])
def test_external_releases_node_on_fault(tmp_path, simulated_node, reference):
    connection, rail = simulated_node
    with external_process(tmp_path, connection, LIMITS + MAPPING) as (process, ready):
        endpoint = ready_endpoint(process, ready)
        with socket.create_connection((endpoint["host"], endpoint["port"]), timeout=10) as peer:
            with peer.makefile("rwb", buffering=0) as stream:
                authenticate(stream, endpoint)
                read_state(stream, step=0)
                assert rail.snapshot()["control"] is not None
                if reference is not None:
                    stream.write(reference)
                    assert stream.readline() == b"END FAULT RAIL_ENVELOPE\n"
        _, summary = finished_run(process, ready, endpoint, status=1)
        assert summary["outcome"] == "FAULT"
        assert summary["reason"] == ("RAIL_ENVELOPE" if reference is not None else "EXTERNAL_DISCONNECTED")
        assert rail.snapshot()["control"] is None
        assert rail.snapshot()["position"]["x"] == pytest.approx(1.0)


def test_node_separates_apply_and_watchdog_deadlines(simulated_node):
    connection, rail = simulated_node
    with NodeClient.from_connection(connection) as client:
        assert client.capabilities()["setpoint_watchdog_deadline"] is True
        client.acquire("stream", ["x"], timeout_s=2.0)
        target = {"x": 1.01}
        now = client.clock()
        client.setpoint(target, 0.025, deadline=now + 0.75)
        assert rail.stream_deadline == now + 0.75

        now = client.clock()
        client.setpoint(target, 0.025, deadline=now + 0.75, watchdog_deadline=now + 1.3)
        assert rail.stream_deadline == now + 1.3

        with pytest.raises(NodeError, match="watchdog"):
            client.setpoint(target, 0.025, deadline=now + 0.75, watchdog_deadline=now + 10.0)
        assert rail.stream_deadline == now + 1.3
