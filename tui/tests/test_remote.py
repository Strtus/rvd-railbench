import copy
import io
import json
import subprocess
import threading
from pathlib import Path

import pytest
from rich.console import Console
from textual.widgets import Button, Input, Select, Static

from conftest import ROOT

pytest.importorskip("rvd_railnode")
from rvd_railnode.node import Node, Settings  # noqa: E402
from rvd_railnode.profile import SIMULATOR, validate  # noqa: E402
from rvd_railnode.server import NodeServer, UserStore  # noqa: E402
from rvd_railnode.sim import SimBackend  # noqa: E402

from rvdlab.app import RvdLabApp  # noqa: E402
from rvdlab.project import find_build  # noqa: E402
from rvdlab.remote import Bridge, connect, connection_path, move, remote_override  # noqa: E402

START = (18.0, 0.03, -0.02)


@pytest.fixture
def node(tmp_path: Path):
    profile = copy.deepcopy(SIMULATOR)
    for axis, value in zip("xyz", START):
        profile["axes"][axis]["start"] = value
    rail = Node(SimBackend(validate(profile), True), Settings(poll_s=0.05))
    (tmp_path / "users.json").write_text(json.dumps({"alice": "a" * 40}))
    server = NodeServer(("127.0.0.1", 0), rail, UserStore(tmp_path / "users.json"))
    rail.start()
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
    thread.start()
    connection = tmp_path / "connection.json"
    connection.write_text(json.dumps({"url": f"http://127.0.0.1:{server.server_address[1]}", "token": "a" * 40}))
    yield connection, rail
    server.shutdown()
    rail.close()
    server.server_close()


def pos(reply: str) -> list[float]:
    words = reply.split()
    assert words[0] == "POS" and len(words) == 11, reply
    return [float(w) for w in words[1:]]


def test_the_bridge_reads_commands_and_holds(node):
    connection, rail = node
    bridge = Bridge(connect(connection))
    bridge.acquire()
    values = pos(bridge.answer(["READ"]))
    assert values[:3] == pytest.approx(START) and values[3:6] == values[:3]
    assert 0 <= values[6] < 0.5 and values[9] == 0
    assert bridge.answer("SEND 0 18.1 0.03 -0.02 0.1 0 0".split()) == "ACK 0"
    assert rail.snapshot()["target"]["x"] == pytest.approx(18.1)
    assert rail.snapshot()["speed"]["y"] == pytest.approx(rail.axes["y"].min_speed)
    assert bridge.answer(["HOLD"]) == "OK"
    assert rail.snapshot()["control"] is None
    bridge.close()


def test_a_stop_reaches_the_node_at_once_and_rail_main_on_its_next_request(node):
    connection, rail = node
    bridge = Bridge(connect(connection))
    bridge.acquire()
    bridge.answer("SEND 0 18.2 0.03 -0.02 0.2 0 0".split())
    bridge.stop()
    assert rail.snapshot()["reason"] == "operator_stop:alice"
    assert pos(bridge.answer(["READ"]))[9] == 1
    assert bridge.answer("SEND 1 18.3 0.03 -0.02 0.2 0 0".split()) == "ACK 1"
    bridge.close()


def test_errors_are_reported_as_lines(node):
    connection, _ = node
    bridge = Bridge(connect(connection))
    bridge.acquire()
    assert bridge.answer("SEND 0 99 0 0 0.1 0 0".split()).startswith("ERR x target 99 is outside")
    assert bridge.answer(["NOPE"]) == "ERR unknown request"
    bridge.close()


def test_move_runs_one_waypoint_and_releases(node):
    connection, rail = node
    assert move(connect(connection), {"x": 18.05}, 0.2, threading.Event()) == "completed"
    state = rail.snapshot()
    assert state["position"]["x"] == pytest.approx(18.05, abs=0.001) and state["control"] is None


@pytest.mark.skipif(find_build(ROOT).rail_main is None, reason="rail_main is not built")
def test_rail_main_runs_through_the_bridge(node, tmp_path):
    connection, rail = node
    bridge = Bridge(connect(connection))
    bridge.acquire()
    bridge.start()
    override = tmp_path / "remote.ini"
    override.write_text(remote_override(3))
    model = ROOT / "policies" / "E_PFCR-DQP_s41_step5120000"
    result = subprocess.run([str(find_build(ROOT).rail_main), "--config", str(ROOT / "configs" / "rail.ini"),
                             "--config", str(override), "--model-dir", str(model), "--bridge", str(bridge.port),
                             "--experiment-dir", str(tmp_path / "run"), "--log", str(tmp_path / "run.csv")],
                            cwd=ROOT, capture_output=True, text=True, timeout=60)
    assert bridge.finished.wait(5)
    bridge.close()
    summary = json.loads((tmp_path / "run" / "summary.json").read_text())
    assert summary["reason"] == "MAX_STEPS", (summary, result.stdout, result.stderr)
    assert json.loads((tmp_path / "run" / "metadata.json").read_text())["hardware"] is True
    state = rail.snapshot()
    assert state["control"] is None and state["position"]["x"] != pytest.approx(START[0], abs=1e-6)


def test_connection_lookup(tmp_path, monkeypatch):
    monkeypatch.setenv("RVDLAB_HOME", str(tmp_path))
    monkeypatch.delenv("RVDLAB_NODE", raising=False)
    assert connection_path() is None
    (tmp_path / "node").mkdir()
    (tmp_path / "node" / "connection.json").write_text("{}")
    assert connection_path() == tmp_path / "node" / "connection.json"
    monkeypatch.setenv("RVDLAB_NODE", str(tmp_path / "other.json"))
    assert connection_path() == tmp_path / "other.json"
    assert connection_path(Path("given.json")) == Path("given.json")


async def settle(pilot, condition, seconds: float = 30.0) -> None:
    for _ in range(int(seconds * 10)):
        await pilot.pause(0.1)
        if condition():
            return
    raise AssertionError("condition not reached")


def shown(widget) -> str:
    console = Console(width=200, file=io.StringIO(), record=True)
    console.print(widget.content)
    return console.export_text()


async def confirm(pilot, app) -> None:
    await settle(pilot, lambda: type(app.screen).__name__ == "ConfirmModal")
    app.screen.query_one("#confirm-yes", Button).press()
    await pilot.pause()


async def test_motion_control_shows_the_node_and_moves_the_rail(node, tmp_path, profiles):
    connection, rail = node
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles, node=connection)
    async with app.run_test(size=(140, 60)) as pilot:
        await pilot.press("2", "enter")
        screen = app.screen
        await settle(pilot, lambda: "simulator" in shown(screen.query_one("#node-view", Static)))
        screen.query_one("#by-x", Input).value = "0.01"
        screen.query_one("#move-speed", Input).value = "0.1"
        screen.query_one("#node-move-by", Button).press()
        await confirm(pilot, app)
        await settle(pilot, lambda: "move completed" in shown(screen.query_one("#node-message", Static)))
        state = rail.snapshot()
        assert state["position"]["x"] == pytest.approx(START[0] + 0.01, abs=0.001) and state["control"] is None


async def test_rendezvous_offers_the_node_and_stops_the_rail(node, tmp_path, profiles):
    connection, rail = node
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles, node=connection)
    async with app.run_test(size=(140, 60)) as pilot:
        await pilot.press("3", "enter")
        screen = app.screen
        screen.query_one("#target", Select).value = "node"
        await pilot.pause()
        assert screen.query_one("#mode", Select).disabled and not screen.query_one("#to-start", Button).disabled
        await pilot.press("f9")
        await settle(pilot, lambda: rail.snapshot()["reason"] == "operator_stop:alice")


@pytest.mark.skipif(find_build(ROOT).rail_main is None, reason="rail_main is not built")
async def test_a_run_on_the_node_ends_with_the_rail_released(node, tmp_path, profiles):
    connection, rail = node
    app = RvdLabApp(root=ROOT, experiments=tmp_path / "experiments", profiles=profiles, node=connection)
    async with app.run_test(size=(140, 60)) as pilot:
        await pilot.press("3", "enter")
        screen = app.screen
        screen.query_one("#target", Select).value = "node"
        screen.query_one("#max-steps", Input).value = "2"
        await pilot.press("f5")
        await confirm(pilot, app)
        await settle(pilot, lambda: screen.outcome is not None, 60)
        assert screen.outcome[0] == "LIMIT_REACHED", screen.outcome
        assert rail.snapshot()["control"] is None and screen.bridge is None
