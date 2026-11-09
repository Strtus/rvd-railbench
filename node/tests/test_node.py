from __future__ import annotations

import threading
import time

import pytest
from conftest import wait_for

from rvd_railnode.backend import InterlockError
from rvd_railnode.node import Rejected


def stream(node, axes=("x", "y"), timeout_s=0.3, user="alice"):
    return node.acquire(user, {"mode": "stream", "axes": list(axes), "period_s": 0.1, "timeout_s": timeout_s})


def setpoint(node, lease, seq, position, speed, valid_for=0.25, user="alice"):
    return node.setpoint(user, {"lease_id": lease["lease_id"], "seq": seq, "deadline": node.clock() + valid_for,
                                "position": position, "speed": speed})


def task(node, points, axes=("x", "y"), user="alice"):
    lease = node.acquire(user, {"mode": "task", "axes": list(axes), "period_s": 0.1})
    node.submit_task(user, {"lease_id": lease["lease_id"], "points": points})
    return lease


class Heartbeat:
    def __init__(self, node, lease, user="alice"):
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, args=(node, lease, user), daemon=True)

    def run(self, node, lease, user):
        while not self.stop.wait(0.05):
            try:
                node.heartbeat(user, {"lease_id": lease["lease_id"]})
            except Rejected:
                return

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *_):
        self.stop.set()
        self.thread.join()


def test_control_is_exclusive_and_needs_its_secret(make_node):
    node = make_node()
    lease = stream(node)
    with pytest.raises(Rejected, match="held by alice") as err:
        stream(node, user="bob")
    assert err.value.status == 409
    with pytest.raises(Rejected):
        node.heartbeat("bob", {"lease_id": lease["lease_id"]})
    with pytest.raises(Rejected):
        node.heartbeat("alice", {"lease_id": "wrong"})
    assert node.heartbeat("alice", {"lease_id": lease["lease_id"]})["ttl_s"] > 0


def test_read_only_node_refuses_control_and_never_moves(make_node):
    node = make_node(allow_motion=False)
    with pytest.raises(Rejected, match="read-only"):
        stream(node)
    assert node.stop("bob")["state"] == "idle"
    assert node.backend.commands == 0 and not node.backend.active


def test_axes_marked_read_only_refuse_control(make_node):
    from conftest import sim_profile

    profile = sim_profile(y={"minimum": -0.5, "maximum": 0.5, "max_speed": 0.5, "tolerance": 0.001, "motion": False})
    node = make_node(profile=profile)
    with pytest.raises(Rejected, match="does not accept motion"):
        stream(node)


def test_stream_moves_with_per_axis_speeds_then_stops_when_setpoints_cease(make_node):
    node = make_node()
    lease = stream(node)
    ack = setpoint(node, lease, 1, {"x": 0.2, "y": 0.05}, {"x": 1.0, "y": 0.2})
    assert ack["accepted_seq"] == 1 and ack["t_applied"] >= ack["t_received"]
    assert node.backend.speed == {"x": 1.0, "y": 0.2, "z": 0.0}
    wait_for(lambda: node.snapshot()["reason"] == "stream_timeout")
    state = node.snapshot()
    assert state["control"] is None and state["fault"] is None and not node.backend.active


@pytest.mark.parametrize("change, message", [
    ({"position": {"x": 11.0}}, "outside"),
    ({"speed": 5.0}, "speed 5 is outside"),
    ({"position": {"x": 3.0}}, "horizon"),
    ({"position": {"z": 0.3}}, "not part of this lease"),
    ({"seq": "2"}, "seq"),
    ({"deadline": 1e9}, "deadline"),
    ({"speed": {"x": 1.0, "y": 1.0}}, "one value per axis"),
    ({"position": {"x": float("nan")}}, "finite"),
    ({"sample_time": -100.0}, "old"),
])
def test_invalid_setpoints_are_rejected_before_any_command(make_node, change, message):
    node = make_node()
    lease = stream(node, timeout_s=0.5)
    body = {"lease_id": lease["lease_id"], "seq": 1, "deadline": node.clock() + 0.4, "position": {"x": 0.1},
            "speed": 1.0, **change}
    with pytest.raises(Rejected, match=message):
        node.setpoint("alice", body)
    assert node.backend.commands == 0


def test_lease_expires_without_heartbeats_and_motion_stops(make_node):
    node = make_node()
    task(node, [{"position": {"x": 8.0}, "speed": 0.5}], axes=["x"])
    wait_for(lambda: node.snapshot()["reason"] == "lease_expired")
    state = node.snapshot()
    assert state["control"] is None and state["job"]["status"] == "stopped" and not node.backend.active


def test_task_never_commands_beyond_its_lead_and_completes(make_node):
    node = make_node()
    gaps = []
    command = node.backend.command

    def watch(targets, speeds):
        gaps.extend(abs(t - node.backend.position[n]) / speeds[n] for n, t in targets.items())
        command(targets, speeds)

    node.backend.command = watch
    points = [{"position": {"x": 1.5, "y": 0.2}, "speed": {"x": 1.0, "y": 0.4}}, {"position": {"x": 0.5}, "speed": 2.0}]
    lease = task(node, points)
    with Heartbeat(node, lease):
        wait_for(lambda: node.snapshot()["job"]["status"] == "completed", timeout=8)
    assert len(gaps) > 5 and max(gaps) <= node.settings.task_lead_s + 0.05
    assert abs(node.backend.position["x"] - 0.5) <= 0.001 and abs(node.backend.position["y"] - 0.2) <= 0.001


def test_any_user_can_stop_and_control_is_revoked(make_node):
    node = make_node()
    lease = task(node, [{"position": {"x": 5.0}, "speed": 1.0}], axes=["x"])
    wait_for(lambda: node.backend.position["x"] > 0.05)
    state = node.stop("bob")
    assert state["reason"] == "operator_stop:bob" and state["control"] is None
    assert state["job"]["status"] == "stopped" and not node.backend.active
    with pytest.raises(Rejected):
        node.heartbeat("alice", {"lease_id": lease["lease_id"]})


def test_release_stops_and_frees_control(make_node):
    node = make_node()
    lease = stream(node, timeout_s=0.5)
    setpoint(node, lease, 1, {"x": 0.3}, 1.0)
    assert node.release("alice", {"lease_id": lease["lease_id"]}) == {"released": True}
    assert node.snapshot()["control"] is None and not node.backend.active
    stream(node, user="bob")


def test_a_stuck_axis_faults_the_node(make_node):
    node = make_node()
    node.backend.stuck.add("x")
    lease = task(node, [{"position": {"x": 5.0}, "speed": 1.0}], axes=["x"])
    with Heartbeat(node, lease):
        wait_for(lambda: node.snapshot()["fault"])
    assert node.snapshot()["fault"].startswith("MOTION_STALLED: x")
    assert not node.backend.active
    with pytest.raises(Rejected, match="faulted"):
        stream(node)


def test_lost_feedback_while_moving_faults_and_stops(make_node):
    node = make_node()
    lease = task(node, [{"position": {"x": 5.0}, "speed": 1.0}], axes=["x"])
    with Heartbeat(node, lease):
        wait_for(lambda: node.backend.position["x"] > 0.05)
        node.backend.fail_reads = 10**6
        wait_for(lambda: node.snapshot()["fault"])
    assert node.snapshot()["fault"].startswith("FEEDBACK_LOST: simulated feedback failure")
    assert not node.backend.active


def test_one_lost_sample_is_tolerated(make_node):
    node = make_node()
    lease = task(node, [{"position": {"x": 5.0}, "speed": 1.0}], axes=["x"])
    with Heartbeat(node, lease):
        wait_for(lambda: node.backend.position["x"] > 0.05)
        node.backend.fail_reads = 1
        time.sleep(0.3)
        assert node.snapshot()["fault"] is None and node.backend.active


def test_an_interlock_stops_at_once(make_node):
    node = make_node()
    lease = task(node, [{"position": {"x": 5.0}, "speed": 1.0}], axes=["x"])
    read = node.backend.read

    def tripped():
        read()
        raise InterlockError("PLC emergency flag M0 is set")

    with Heartbeat(node, lease):
        wait_for(lambda: node.backend.position["x"] > 0.05)
        node.backend.read = tripped
        wait_for(lambda: node.snapshot()["fault"])
    assert node.snapshot()["fault"] == "INTERLOCK: PLC emergency flag M0 is set"
    assert not node.backend.active


def test_slow_arming_neither_trips_the_node_nor_blocks_requests(make_node):
    node = make_node()
    node.backend.enable_delay_s = 0.8
    result = {}
    worker = threading.Thread(target=lambda: result.update(lease=stream(node, axes=["z"], timeout_s=0.5)))
    worker.start()
    time.sleep(0.2)
    begin = time.monotonic()
    assert node.snapshot()["control"]["armed"] is False
    assert time.monotonic() - begin < 0.1
    worker.join()
    ack = setpoint(node, result["lease"], 1, {"z": 0.35}, 0.2)
    assert ack["accepted_seq"] == 1
    time.sleep(0.1)
    assert node.snapshot()["fault"] is None


def test_the_control_thread_survives_unexpected_errors(make_node):
    node = make_node()
    lease = stream(node, timeout_s=0.5)

    def broken(targets, speeds):
        raise KeyError("driver bug")

    node.backend.command = broken
    with pytest.raises(Rejected, match="command failed"):
        setpoint(node, lease, 1, {"x": 0.1}, 1.0)
    assert node.snapshot()["fault"].startswith("COMMAND_FAILED")
    assert node._thread.is_alive()
    with pytest.raises(Rejected, match="node computer"):
        node.reset("alice", local=False)
    assert node.reset("alice", local=True)["fault"] is None
    stream(node)


def test_an_unconfirmed_stop_latches_and_is_retried(make_node):
    node = make_node()
    lease = stream(node, timeout_s=0.5)
    setpoint(node, lease, 1, {"x": 0.3}, 1.0)
    node.backend.fail_stop = True
    with pytest.raises(Rejected, match="STOP_UNCONFIRMED"):
        node.stop("bob")
    assert node.backend.active
    node.backend.fail_stop = False
    wait_for(lambda: not node.backend.active, timeout=3)
    assert "STOP_UNCONFIRMED" in node.snapshot()["fault"]
    with pytest.raises(Rejected, match="faulted"):
        stream(node)


def test_settings_reject_a_lead_beyond_the_horizon():
    from rvd_railnode.node import Settings

    with pytest.raises(ValueError, match="horizon"):
        Settings.from_profile({}, {"task_lead_s": 5.0, "horizon_s": 3.0})
    with pytest.raises(ValueError, match="unknown"):
        Settings.from_profile({}, {"speed_of_light": 1.0})


def test_a_fresh_snapshot_polls_at_once(make_node):
    node = make_node(poll_s=5.0)
    before = node.snapshot()["sensors"]["encoders"]["seq"]
    asked = node.clock()
    state = node.snapshot(fresh=True)
    assert state["sensors"]["encoders"]["sample_time"] >= asked
    assert state["sensors"]["encoders"]["seq"] == before + 1
    assert node.clock() - asked < 1.0


def test_a_task_from_outside_the_limits_never_commands_beyond_them(make_node):
    node = make_node()
    node.backend.position["y"] = 0.7  # the axis rests outside its configured range
    commanded = []
    command = node.backend.command

    def watch(targets, speeds):
        commanded.extend(targets.values())
        command(targets, speeds)

    node.backend.command = watch
    lease = task(node, [{"position": {"y": 0.0}, "speed": 0.2}], axes=["y"])
    with Heartbeat(node, lease):
        wait_for(lambda: node.snapshot()["job"]["status"] == "completed", timeout=8)
    assert commanded and max(commanded) <= 0.5 and node.snapshot()["fault"] is None


def test_held_axes_stay_enabled_through_an_ordinary_stop_but_not_a_fault(make_node):
    from conftest import sim_profile

    profile = sim_profile(z={"minimum": 0.0, "maximum": 0.6, "max_speed": 0.5, "acceleration": 50.0,
                             "tolerance": 0.001, "start": 0.3, "hold": True})
    node = make_node(profile=profile)
    lease = task(node, [{"position": {"z": 0.32}, "speed": 0.2}], axes=["z"])
    with Heartbeat(node, lease):
        wait_for(lambda: node.snapshot()["job"]["status"] == "completed")
    node.release("alice", {"lease_id": lease["lease_id"]})
    state = node.snapshot()
    assert state["state"] == "holding" and state["armed"] == ["z"] and node.backend.armed == {"z"}
    stream(node, axes=["x"])  # others can take the rail meanwhile
    node.stop("bob")
    assert node.backend.armed == {"z"}
    node.backend.fail_reads = 10**6
    wait_for(lambda: node.snapshot()["fault"])
    wait_for(lambda: not node.backend.active)
    assert node.backend.armed == set()


def test_remote_reset_when_the_node_allows_it(make_node):
    node = make_node()
    node.remote_reset = True
    lease = stream(node, timeout_s=0.5)

    def broken(targets, speeds):
        raise KeyError("driver bug")

    node.backend.command = broken
    with pytest.raises(Rejected):
        setpoint(node, lease, 1, {"x": 0.1}, 1.0)
    assert node.reset("alice", local=False)["fault"] is None


def test_a_stalled_event_consumer_does_not_hold_up_control(make_node):
    node = make_node()
    stuck = threading.Event()
    node.on_event = lambda kind, detail: stuck.wait(30)
    begin = time.monotonic()
    lease = stream(node)
    setpoint(node, lease, 1, {"x": 0.1}, 1.0)
    node.stop("bob")
    assert time.monotonic() - begin < 3.0
    stuck.set()
