"""Read offline comparison output; all positions are Hill-frame metres."""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path

from .telemetry import num


@dataclass
class Comparison:
    nominal: list[dict[str, str]] = field(default_factory=list)
    events: dict[int, list[dict[str, str]]] = field(default_factory=dict)
    actual: list[dict[str, str]] = field(default_factory=list)

    @classmethod
    def load(cls, output: Path, experiment: Path) -> Comparison:
        def read(path: Path) -> list[dict[str, str]]:
            with path.open(encoding="utf-8", newline="") as stream:
                return list(csv.DictReader(stream))

        result = cls(nominal=read(output / "nominal.csv"), actual=read(experiment / "trajectory.csv"))
        for row in read(output / "local.csv"):
            result.events.setdefault(int(row["event_k"]), []).append(row)
        return result

    def strongest_event(self) -> int | None:
        candidates = [row for row in self.actual if int(row["k"]) in self.events]
        if not candidates:
            return None
        row = max(candidates, key=lambda r: sum(
            ((num(r, f"ud{i}") or 0) - (num(r, f"ua{i}") or 0)) ** 2 for i in range(3)))
        return int(row["k"])


def coordinates(rows: list[dict[str, str]], x: str, y: str) -> tuple[list[float], list[float]]:
    pairs = [(a, b) for row in rows if (a := num(row, x)) is not None and (b := num(row, y)) is not None]
    return [a for a, _ in pairs], [b for _, b in pairs]
