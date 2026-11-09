from __future__ import annotations

import json
import re
import socket
import threading

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


def test_status_and_stop_reach_the_local_node(initialized, capsys):
    root, _ = initialized
    config_path = root / "node.json"
    config = json.loads(config_path.read_text())
    config["host"] = "127.0.0.1"
    config_path.write_text(json.dumps(config))
    server = build(load_config(config_path))
    server.node.start()
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
    thread.start()
    try:
        wait_for(lambda: server.node.snapshot()["sensors"]["encoders"]["ok"])
        assert main(["status", "--config", str(config_path)]) == 0
        out = capsys.readouterr().out
        assert "simulator" in out and "read-only" in out and re.search(r"^x\s+0\.000000", out, re.M)
        assert main(["stop", "--config", str(config_path)]) == 0
        assert "stopped" in capsys.readouterr().out
    finally:
        server.shutdown()
        server.node.close()
        server.server_close()


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
