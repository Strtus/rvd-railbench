from __future__ import annotations

import copy
import json
import re
import socket
import threading
from contextlib import contextmanager

import pytest
from conftest import ROOT, sim_profile, wait_for

import rvd_railnode
from rvd_railnode.cli import main
from rvd_railnode.config import load as load_config
from rvd_railnode.server import build


def free_port():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


@pytest.fixture
def initialized(tmp_path):
    pytest.importorskip("cryptography")
    root = tmp_path / "node"
    port = free_port()
    assert main(["init", "--directory", str(root), "--address", "10.0.0.5", "--port", str(port),
                 "--users", "alice", "bob"]) == 0
    return root, port


def test_init_creates_a_complete_node_directory(initialized):
    from cryptography import x509

    root, port = initialized
    config = json.loads((root / "node.json").read_text())
    assert config["allow_motion"] is False and config["port"] == port
    assert json.loads((root / "profile.json").read_text())["driver"] == "simulator"
    assert set(json.loads((root / "users.json").read_text())) == {"local", "alice", "bob"}
    local = json.loads((root / "access" / "local" / "connection.json").read_text())
    alice = json.loads((root / "access" / "alice" / "connection.json").read_text())
    assert local["url"] == f"https://127.0.0.1:{port}" and alice["url"] == f"https://10.0.0.5:{port}"
    assert (root / "access" / "alice" / "node-ca.pem").read_bytes() == (root / "node-ca.pem").read_bytes()
    cert = x509.load_pem_x509_certificate((root / "node-cert.pem").read_bytes())
    names = cert.extensions.get_extension_for_class(x509.SubjectAlternativeName).value
    assert {str(ip) for ip in names.get_values_for_type(x509.IPAddress)} == {"10.0.0.5", "127.0.0.1"}
    assert names.get_values_for_type(x509.DNSName) == ["localhost"]


def test_init_never_overwrites_and_reserves_the_local_user(initialized, tmp_path, capsys):
    root, _ = initialized
    assert main(["init", "--directory", str(root), "--address", "10.0.0.5"]) == 2
    assert main(["init", "--directory", str(tmp_path / "other"), "--address", "10.0.0.5", "--users", "local"]) == 2
    assert "reserved" in capsys.readouterr().err


def test_users_can_be_added_and_removed(initialized):
    root, port = initialized
    assert main(["add-user", "carol", "--config", str(root / "node.json")]) == 0
    carol = json.loads((root / "access" / "carol" / "connection.json").read_text())
    assert carol["url"] == f"https://10.0.0.5:{port}"
    assert carol["token"] == json.loads((root / "users.json").read_text())["carol"]
    assert main(["add-user", "carol", "--config", str(root / "node.json")]) == 2
    assert main(["remove-user", "carol", "--config", str(root / "node.json")]) == 0
    assert "carol" not in json.loads((root / "users.json").read_text())
    assert not (root / "access" / "carol").exists()


@contextmanager
def running(root, allow_motion=False):
    config_path = root / "node.json"
    config = json.loads(config_path.read_text())
    config["host"] = "127.0.0.1"
    config_path.write_text(json.dumps(config))
    server = build(load_config(config_path), allow_motion)
    server.node.start()
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
    thread.start()
    try:
        wait_for(lambda: server.node.snapshot()["sensors"]["encoders"]["ok"])
        yield config_path, server.node
    finally:
        server.shutdown()
        server.node.close()
        server.server_close()


def test_status_and_stop_reach_the_local_node(initialized, capsys):
    with running(initialized[0]) as (config_path, _):
        assert main(["status", "--config", str(config_path)]) == 0
        out = capsys.readouterr().out
        assert "simulator" in out and "read-only" in out and re.search(r"^x\s+0\.000000", out, re.M)
        assert main(["stop", "--config", str(config_path)]) == 0
        assert "stopped" in capsys.readouterr().out


def test_status_lists_the_plc_diagnostics_on_request(tmp_path, capsys):
    pytest.importorskip("cryptography")
    from plc_emulator import PlcEmulator
    from test_plc import PROFILE

    profile = copy.deepcopy(PROFILE)
    profile["diagnostics"] = {"z_error": 30, "moves": 31}
    with PlcEmulator(profile) as plc:
        plc.set_d(30, -20)
        profile["modbus"].update(host="127.0.0.1", port=plc.port, timeout_s=0.5)
        (tmp_path / "plc.json").write_text(json.dumps(profile))
        root = tmp_path / "node"
        assert main(["init", "--directory", str(root), "--address", "10.0.0.5", "--port", str(free_port()),
                     "--profile", str(tmp_path / "plc.json")]) == 0
        capsys.readouterr()
        with running(root) as (config_path, _):
            assert main(["status", "--config", str(config_path)]) == 0
            assert "diagnostics" not in capsys.readouterr().out
            assert main(["status", "--config", str(config_path), "--diagnostics"]) == 0
            out = capsys.readouterr().out
            assert re.search(r"^diagnostics$", out, re.M) and re.search(r"z_error\s+-20\s+moves\s+0", out)


def test_status_says_when_the_profile_has_no_diagnostics(initialized, capsys):
    with running(initialized[0]) as (config_path, _):
        assert main(["status", "--config", str(config_path), "--diagnostics"]) == 0
        assert "none in the profile" in capsys.readouterr().out


def test_move_goes_there_and_releases_control(initialized, capsys):
    with running(initialized[0], allow_motion=True) as (config_path, node):
        assert main(["move", "--config", str(config_path), "--by", "x=0.02", "z=-0.01", "--speed", "0.1"]) == 0
        assert "arrived" in capsys.readouterr().out
        state = node.snapshot()
        assert state["position"]["x"] == pytest.approx(0.02, abs=0.001)
        assert state["position"]["z"] == pytest.approx(-0.01, abs=0.001)
        assert state["control"] is None and not node.backend.active
        assert main(["move", "--config", str(config_path), "--to", "x=1", "--by", "x=0.1", "--speed", "0.1"]) == 2
        assert main(["move", "--config", str(config_path), "--by", "x=0.1", "--speed", "9"]) == 2
        assert "speed 9 is outside" in capsys.readouterr().err


def test_park_drives_the_held_axes_to_rest(initialized, capsys):
    root = initialized[0]
    profile = sim_profile(z={"minimum": -0.5, "maximum": 0.5, "max_speed": 0.5, "acceleration": 50.0,
                             "tolerance": 0.001, "hold": True, "park": -0.2, "park_speed": 0.4})
    (root / "profile.json").write_text(json.dumps(profile))
    with running(root, allow_motion=True) as (config_path, node):
        assert main(["park", "--config", str(config_path)]) == 0
        assert "nothing to park" in capsys.readouterr().out
        assert main(["move", "--config", str(config_path), "--to", "z=0.05", "--speed", "0.2"]) == 0
        assert node.snapshot()["state"] == "holding"
        assert main(["park", "--config", str(config_path)]) == 0
        assert "parked and released" in capsys.readouterr().out
        state = node.snapshot()
        assert state["position"]["z"] == pytest.approx(-0.2, abs=0.002) and state["state"] == "idle"


def test_probe_reads_once_and_prints_positions(tmp_path, capsys):
    path = tmp_path / "sim.json"
    path.write_text(json.dumps(sim_profile()))
    assert main(["probe", "--profile", str(path)]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["position"] == {"x": 0.0, "y": 0.0, "z": 0.3}


def test_a_broken_profile_is_reported_plainly(tmp_path, capsys):
    path = tmp_path / "bad.json"
    path.write_text(json.dumps({"driver": "modbus", "axes": {"q": {}}}))
    assert main(["probe", "--profile", str(path)]) == 2
    assert "axes must be a non-empty subset" in capsys.readouterr().err


def test_the_node_version_follows_the_project_version():
    text = (ROOT / "CMakeLists.txt").read_text()
    assert rvd_railnode.__version__ == re.search(r"project\(rvd_railbench VERSION ([0-9.]+)", text).group(1)
