"""Backends share one small interface; only the node's control thread calls them."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable


class BackendError(RuntimeError):
    pass


class InterlockError(BackendError):
    """The device reports a state in which no motion may continue (wrong program, emergency flag)."""


@dataclass
class Sample:
    positions: dict[str, float]
    time: float  # node clock when the read started
    duration: float
    raw: dict[str, int] = field(default_factory=dict)


def create(profile: dict[str, Any], allow_motion: bool, clock: Callable[[], float] = time.monotonic) -> Any:
    if profile["driver"] == "modbus":
        from .plc import PlcBackend

        return PlcBackend(profile, allow_motion, clock=clock)
    from .sim import SimBackend

    return SimBackend(profile, allow_motion, clock=clock)
