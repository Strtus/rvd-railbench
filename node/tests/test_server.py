from __future__ import annotations

import json
import socket
import ssl
import threading
import time
import urllib.error
import urllib.request

import pytest
from conftest import FAST, sim_profile, wait_for

from rvd_railnode.client import NodeClient, NodeError
from rvd_railnode.node import Node, Settings
from rvd_railnode.server import NodeServer, UserStore, tls_context
from rvd_railnode.sim import SimBackend

pytest.importorskip("cryptography")
from rvd_railnode.cli import _identity  # noqa: E402

TOKENS = {"alice": "a" * 40, "bob": "b" * 40}


@pytest.fixture
def served(tmp_path):
    ca, _, cert, key, _ = _identity("127.0.0.1")
    (tmp_path / "ca.pem").write_bytes(ca)
    (tmp_path / "cert.pem").write_bytes(cert)
    (tmp_path / "key.pem").write_bytes(key)
    (tmp_path / "users.json").write_text(json.dumps(TOKENS))
    node = Node(SimBackend(sim_profile(), True), Settings(**{**FAST, "lease_ttl_s": 1.0}))
    server = NodeServer(("127.0.0.1", 0), node, UserStore(tmp_path / "users.json"), tmp_path / "audit.jsonl",
                        tls_context(tmp_path / "cert.pem", tmp_path / "key.pem"), io_timeout_s=2.0)
    node.start()
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
    thread.start()
    wait_for(lambda: node.snapshot()["sensors"]["encoders"]["ok"])
    yield f"https://127.0.0.1:{server.server_address[1]}", tmp_path, server
    server.shutdown()
    node.close()
    server.server_close()


def connect(served, user="alice"):
    url, root, _ = served
    return NodeClient(url, TOKENS[user], root / "ca.pem")


def test_requests_need_a_valid_token(served):
    url, root, _ = served
    with pytest.raises(NodeError) as err:
        NodeClient(url, "x" * 40, root / "ca.pem").state()
    assert err.value.status == 401
    request = urllib.request.Request(url + "/v1/state")
    context = ssl.create_default_context(cafile=str(root / "ca.pem"))
    with pytest.raises(urllib.error.HTTPError) as raw:
        urllib.request.urlopen(request, timeout=2, context=context)
    assert raw.value.code == 401
    raw.value.close()


def test_the_node_certificate_must_be_trusted(served):
    url, _, _ = served
    with pytest.raises(NodeError, match="CERTIFICATE_VERIFY_FAILED|certificate verify failed"):
        NodeClient(url, TOKENS["alice"]).state()


def test_stream_control_end_to_end(served):
    with connect(served) as client:
        assert client.capabilities()["axes"]["x"]["max_speed"] == 2.0
        client.acquire("stream", ["x"], period_s=0.1, timeout_s=0.5)
        assert client.setpoint({"x": 0.2}, {"x": 1.0})["accepted_seq"] == 1
        assert client.state()["control"]["owner"] == "alice"
        with pytest.raises(NodeError, match="held by alice") as busy:
            connect(served, "bob").acquire("stream", ["x"], period_s=0.1)
        assert busy.value.status == 409
    assert connect(served, "bob").state()["control"] is None


def test_task_end_to_end(served):
    with connect(served) as client:
        client.acquire("task", ["x", "y"], period_s=0.1)
        client.task([{"position": {"x": 0.6, "y": -0.1}, "speed": 0.5}])
        wait_for(lambda: client.state()["job"]["status"] == "completed", timeout=8)
        position = client.state()["position"]
    assert position["x"] == pytest.approx(0.6, abs=0.001) and position["y"] == pytest.approx(-0.1, abs=0.001)


def test_an_observer_stop_revokes_control(served):
    with connect(served) as client:
        client.acquire("stream", ["x"], period_s=0.1, timeout_s=0.5)
        client.setpoint({"x": 0.1}, 1.0)
        assert connect(served, "bob").stop()["reason"] == "operator_stop:bob"
        with pytest.raises(NodeError) as err:
            client.setpoint({"x": 0.2}, 1.0)
        assert err.value.status == 409


def test_a_silent_connection_does_not_block_others(served):
    url, _, _ = served
    port = int(url.rsplit(":", 1)[1])
    with socket.create_connection(("127.0.0.1", port)):
        begin = time.monotonic()
        assert connect(served).state()["backend"] == "simulator"
        assert time.monotonic() - begin < 1.0


def test_new_users_are_accepted_without_a_restart(served):
    url, root, _ = served
    (root / "users.json").write_text(json.dumps({**TOKENS, "carol": "c" * 48}))
    assert NodeClient(url, "c" * 48, root / "ca.pem").state()["backend"] == "simulator"


def test_the_audit_log_records_actions_but_no_secrets(served):
    _, root, _ = served
    with connect(served) as client:
        lease = client.acquire("stream", ["x"], period_s=0.1, timeout_s=0.5)
        client.setpoint({"x": 0.1}, 1.0)
    text = (root / "audit.jsonl").read_text()
    paths = [json.loads(line)["path"] for line in text.splitlines()]
    assert "/v1/lease" in paths and "/v1/release" in paths and "/v1/setpoint" not in paths
    assert lease["lease_id"] not in text and TOKENS["alice"] not in text
