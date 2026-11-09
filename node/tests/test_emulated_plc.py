"""The node, the Modbus driver and the wire protocol together, against an emulated positioning PLC."""

from __future__ import annotations

import copy
import time

import pytest
from conftest import wait_for
from plc_emulator import PlcEmulator
from test_plc import PROFILE

from rvd_railnode.node import Node, Rejected, Settings
from rvd_railnode.plc import PlcBackend

START = {"x": 1000, "z": 60000}


@pytest.fixture
def rig():
    profile = copy.deepcopy(PROFILE)
    with PlcEmulator(profile, START) as plc:
        profile["modbus"].update(host="127.0.0.1", port=plc.port, timeout_s=0.5)
        node = Node(PlcBackend(profile, allow_motion=True),
                    Settings(poll_s=0.05, lease_ttl_s=1.0, stall_timeout_s=1.0, max_sample_age_s=1.0,
                             task_lead_s=0.5, task_refresh_s=0.2, horizon_s=1.0))
        node.start()
        wait_for(lambda: node.snapshot()["sensors"]["encoders"]["ok"])
        yield node, plc
        node.close()


def test_stream_setpoints_move_the_emulated_axes_and_release_stops_them(rig):
    node, plc = rig
    assert node.snapshot()["position"] == pytest.approx({"x": 0.1, "z": 0.1})
    lease = node.acquire("alice", {"mode": "stream", "axes": ["x", "z"], "period_s": 0.2, "timeout_s": 1.0})
    assert plc.get_d(23) == 1  # servo enabled while arming, holding its position
    for seq in range(1, 6):
        here = node.snapshot()["position"]
        node.setpoint("alice", {"lease_id": lease["lease_id"], "seq": seq, "deadline": node.clock() + 0.9,
                                "position": {"x": here["x"] + 0.02, "z": here["z"] + 0.004},
                                "speed": {"x": 0.05, "z": 0.01}})
        time.sleep(0.2)
    moved = node.snapshot()["position"]
    assert moved["x"] > 0.13 and moved["z"] > 0.103
    node.release("alice", {"lease_id": lease["lease_id"]})
    assert not node.backend.active and plc.get_d(23) == 0 and plc.get_d(24) == 1
    still = plc.get_d(12)
    time.sleep(0.2)
    assert plc.get_d(12) == still


def test_a_task_reaches_its_waypoint_through_the_driver(rig):
    node, plc = rig
    lease = node.acquire("alice", {"mode": "task", "axes": ["x"], "period_s": 0.2})
    node.submit_task("alice", {"lease_id": lease["lease_id"], "points": [{"position": {"x": 0.16}, "speed": 0.1}]})
    gaps = []

    def done():
        node.heartbeat("alice", {"lease_id": lease["lease_id"]})
        gaps.append(abs(plc.get_d(10) - plc.get_d(12)) * 1e-4 / 0.1)
        return node.snapshot()["job"]["status"] == "completed"

    wait_for(done, timeout=8, step=0.05)
    assert plc.get_d(12) == 1600 and max(gaps) <= 0.5 + 0.1
    node.release("alice", {"lease_id": lease["lease_id"]})


def test_the_emergency_flag_stops_the_node(rig):
    node, plc = rig
    lease = node.acquire("alice", {"mode": "task", "axes": ["x"], "period_s": 0.2})
    node.submit_task("alice", {"lease_id": lease["lease_id"], "points": [{"position": {"x": 0.5}, "speed": 0.1}]})
    wait_for(lambda: plc.get_d(12) > 1050)
    plc.set_m(0, True)
    wait_for(lambda: node.snapshot()["fault"])
    assert node.snapshot()["fault"].startswith("INTERLOCK: PLC emergency flag M0")
    wait_for(lambda: not node.backend.active, timeout=8)
    with pytest.raises(Rejected, match="faulted"):
        node.acquire("bob", {"mode": "task", "axes": ["x"], "period_s": 0.2})
