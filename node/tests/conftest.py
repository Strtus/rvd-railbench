from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Callable

import pytest

from rvd_railnode.node import Node, Settings
from rvd_railnode.profile import validate
from rvd_railnode.sim import SimBackend

ROOT = Path(__file__).resolve().parents[2]

FAST = dict(poll_s=0.01, lease_ttl_s=0.5, max_timeout_s=1.0, max_period_s=0.5, min_period_s=0.01,
            stall_timeout_s=0.3, max_sample_age_s=0.25, task_lead_s=0.2, task_refresh_s=0.05, horizon_s=0.4,
            wait_s=5.0)


def sim_profile(**axes: dict[str, Any]) -> dict[str, Any]:
    base = {
        "x": {"minimum": 0.0, "maximum": 10.0, "max_speed": 2.0, "acceleration": 50.0, "tolerance": 0.001},
        "y": {"minimum": -0.5, "maximum": 0.5, "max_speed": 0.5, "acceleration": 50.0, "tolerance": 0.001},
        "z": {"minimum": 0.0, "maximum": 0.6, "max_speed": 0.5, "acceleration": 50.0, "tolerance": 0.001,
              "start": 0.3},
    }
    base.update(axes)
    return validate({"driver": "simulator", "axes": base})


def wait_for(predicate: Callable[[], Any], timeout: float = 5.0, step: float = 0.01) -> Any:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = predicate()
        if value:
            return value
        time.sleep(step)
    raise AssertionError("condition not reached in time")


@pytest.fixture
def make_node():
    nodes: list[Node] = []

    def make(allow_motion: bool = True, profile: dict[str, Any] | None = None, **settings: Any) -> Node:
        backend = SimBackend(profile or sim_profile(), allow_motion)
        node = Node(backend, Settings(**{**FAST, **settings}))
        node.start()
        wait_for(lambda: node.snapshot()["sensors"]["encoders"]["ok"])
        nodes.append(node)
        return node

    yield make
    for node in nodes:
        node.close()
