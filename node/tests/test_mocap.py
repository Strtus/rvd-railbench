from __future__ import annotations

import json
import math
import struct

import pytest
from conftest import FAST, sim_profile, wait_for
from vrpn_server import FakeVrpn, description, message, pose

from rvd_railnode.cli import main
from rvd_railnode.mocap import Decoder, MocapReceiver, VrpnError
from rvd_railnode.node import Node, Settings
from rvd_railnode.profile import ProfileError, validate
from rvd_railnode.sim import SimBackend

SWAP = {"matrix": [[0, -1, 0], [-1, 0, 0], [0, 0, 1]], "offset": [1.0, 2.0, 3.0]}


def test_the_decoder_reassembles_fragments_and_skips_what_it_does_not_need():
    stream = (description(-1, 3, "Chaser") + description(-1, 4, "目标") + description(-2, 7, "vrpn_Tracker Pos_Quat")
              + description(-2, 8, "vrpn_Tracker Velocity")
              + pose(3, (1.0, 2.0, 3.0), stamp=100.25)
              + message(8, 3, b"\0" * 76, 100.3)
              + pose(3, (math.nan, 0.0, 0.0), stamp=100.4)
              + pose(4, (4.0, 5.0, 6.0), (0.0, 0.0, 0.6, 0.8), stamp=100.5))
    decoder, poses = Decoder(), []
    for byte in stream:
        poses += decoder.feed(bytes([byte]))
    assert [(p[0], p[3]) for p in poses] == [("Chaser", (1.0, 2.0, 3.0)), ("目标", (4.0, 5.0, 6.0))]
    assert poses[0][2] == pytest.approx(100.25) and poses[1][4] == (0.0, 0.0, 0.6, 0.8)
    assert decoder.buffer == b""


def test_the_decoder_rejects_a_corrupt_stream():
    with pytest.raises(VrpnError, match="length"):
        Decoder().feed(b"\xff" * 24)
    with pytest.raises(VrpnError, match="tracker message"):
        Decoder().feed(description(-2, 7, "vrpn_Tracker Pos_Quat") + message(7, 0, struct.pack(">ii3d", 0, 0, 1, 2, 3)))


def receiver(server, **options):
    config = {"host": "127.0.0.1", "port": server.port, "max_age_s": 0.15, "max_repeat_s": 0.2,
              "trackers": {"Chaser": {"to_rail": SWAP}, "Target": {}}, **options}
    return MocapReceiver(config).start()


def valid(mocap, name):
    return mocap.state()["trackers"][name]["valid"]


def test_live_poses_are_valid_and_mapped_to_the_rail_frame():
    with FakeVrpn() as server:
        server.positions["Chaser"] = [0.5, -0.25, 1.0]
        mocap = receiver(server)
        try:
            wait_for(lambda: valid(mocap, "Chaser") and valid(mocap, "Target") and mocap.state()["frames"] > 10)
            state = mocap.state()
            chaser = state["trackers"]["Chaser"]
            assert state["connected"] and state["error"] is None and chaser["frames"] > 5
            assert chaser["rail"] == pytest.approx({"x": 1.25, "y": 1.5, "z": 4.0}, abs=1e-5)
            assert "rail" not in state["trackers"]["Target"]
        finally:
            mocap.close()


def test_a_pose_replayed_bit_for_bit_is_flagged_even_with_fresh_stamps():
    with FakeVrpn() as server:
        mocap = receiver(server)
        try:
            wait_for(lambda: valid(mocap, "Chaser"))
            server.frozen = True
            wait_for(lambda: not valid(mocap, "Chaser"))
            assert "repeated" in mocap.state()["trackers"]["Chaser"]["reason"]
            server.frozen = False
            wait_for(lambda: valid(mocap, "Chaser"))
        finally:
            mocap.close()


def test_silence_goes_stale_and_a_dropped_connection_is_restored():
    with FakeVrpn() as server:
        mocap = receiver(server)
        try:
            wait_for(lambda: valid(mocap, "Chaser"))
            server.silent = True
            wait_for(lambda: not valid(mocap, "Chaser"))
            assert mocap.state()["trackers"]["Chaser"]["reason"].startswith("no pose")
            server.silent = False
            server.drop()
            wait_for(lambda: mocap.state()["connections"] == 2 and valid(mocap, "Chaser"), timeout=8)
        finally:
            mocap.close()


def test_an_unreachable_server_is_reported():
    with FakeVrpn() as server:
        port = server.port
    mocap = MocapReceiver({"host": "127.0.0.1", "port": port, "trackers": {"Chaser": {}}}).start()
    try:
        wait_for(lambda: mocap.state()["error"] not in (None, "not started"))
        state = mocap.state()
        assert not state["connected"] and not state["trackers"]["Chaser"]["valid"]
    finally:
        mocap.close()


def test_the_node_reports_motion_capture():
    with FakeVrpn() as server:
        mocap = MocapReceiver({"host": "127.0.0.1", "port": server.port, "trackers": {"Chaser": {}}})
        node = Node(SimBackend(sim_profile(), True), Settings(**FAST), mocap=mocap)
        node.start()
        try:
            assert node.capabilities()["sensors"] == ["encoders", "mocap"]
            wait_for(lambda: node.snapshot()["sensors"]["mocap"]["trackers"]["Chaser"]["valid"])
        finally:
            node.close()
        assert not mocap._thread.is_alive()


@pytest.mark.parametrize("mocap, message", [
    ({"host": "h"}, "mocap.trackers"),
    ({"host": "h", "trackers": {}}, "at least one"),
    ({"host": "h", "trackers": {"C": {"to_rail": {"matrix": [[1, 0, 0]], "offset": [0, 0, 0]}}}}, "3x3"),
    ({"host": "h", "trackers": {"C": {"to_rail": {"matrix": [[1, 0, 0]] * 3, "offset": [0, 0]}}}}, "three"),
    ({"driver": "optitrack", "host": "h", "trackers": {"C": {}}}, "vrpn"),
])
def test_bad_mocap_sections_are_rejected(mocap, message):
    with pytest.raises(ProfileError, match=message):
        validate({**sim_profile(), "mocap": mocap})


def test_probe_listens_to_motion_capture(tmp_path, capsys):
    with FakeVrpn() as server:
        path = tmp_path / "sim.json"
        path.write_text(json.dumps({**sim_profile(), "mocap": {"host": "127.0.0.1", "port": server.port,
                                                                "trackers": {"Chaser": {}, "Target": {}}}}))
        assert main(["probe", "--profile", str(path), "--mocap-seconds", "0.5"]) == 0
    report = json.loads(capsys.readouterr().out)
    target = report["mocap"]["trackers"]["Target"]
    assert target["valid"] and target["rate_hz"] > 50
