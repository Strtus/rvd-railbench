"""Braille orbital welcome, ported from the supplied rvdlab_braille_anim.ps1.

Preserves its logo geometry, reveal order, ANSI palette and 0.105 rad/frame
comet motion. This is a logo animation, not a physical orbit simulation.
"""

from __future__ import annotations

import math
from functools import lru_cache

from rich.text import Text

from .project import user_data_dir

COLS, ROWS, FPS = 108, 27, 24
REVEAL_FRAMES = 90
ANGLE_STEP = 0.105
ORBIT_FRAMES = math.ceil(6 * math.tau / ANGLE_STEP)
DURATION = (REVEAL_FRAMES + ORBIT_FRAMES) / FPS
BASE_COLOR = "#005faf"
COLORS = ("#ffffff", "#00d7ff", "#00afff", "#0087ff", "#005fff")
BITS = ((1, 2, 4, 64), (8, 16, 32, 128))


def welcome_needed() -> bool:
    try:
        return not (user_data_dir() / "welcome-seen").is_file()
    except OSError:
        return False


def mark_welcome_seen() -> None:
    """A read-only user directory must never prevent entry to the application."""
    try:
        directory = user_data_dir()
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "welcome-seen").touch(exist_ok=True)
    except OSError:
        pass


class OrbitalAnimation:
    def __init__(self, cols: int = COLS, rows: int = ROWS) -> None:
        self.cols, self.rows = cols, rows
        dw, dh = cols * 2, rows * 4
        self.scale = min((dw - 4) / 2201, (dh - 8) / 1206)
        self.ox = (dw - 2201 * self.scale) / 2
        self.oy = (dh - 1206 * self.scale) / 2
        self.orbits = [(1100.5 - 162 * k, (1100.5 - 162 * k) * .545) for k in range(6)]
        dots: set[tuple[int, int]] = set()

        def put(x: float, y: float) -> None:
            dx, dy = self.map_dot(x, y)
            if 0 <= dx < dw and 0 <= dy < dh:
                dots.add((dx, dy))

        for a, b in self.orbits:
            steps = round(max(1400, a * 3))
            for i in range(steps + 1):
                angle = math.tau * i / steps
                put(2340 - a + a * math.cos(angle), 1260 + b * math.sin(angle))
        for i in range(600):
            angle = math.tau * i / 600
            put(2307.5 + 32.5 * math.cos(angle), 1260 + 32.5 * math.sin(angle))

        apex_x = (2340 - 139) * self.scale + self.ox
        apex_y = (1260 - 657) * self.scale + self.oy
        distances = {(x, y): math.hypot(x - apex_x, y - apex_y) for x, y in dots}
        maximum = max(1.0, *distances.values())
        bins: list[list[tuple[int, int]]] = [[] for _ in range(REVEAL_FRAMES)]
        for point, distance in distances.items():
            bins[min(89, round((distance / maximum)**.92 * 89))].append(point)
        masks = [0] * (cols * rows)
        self.reveal: list[tuple[tuple[int, ...], dict[int, int]]] = []
        for group in bins:
            if not group:
                continue
            hot = {}
            for x, y in group:
                index = y // 4 * cols + x // 2
                bit = BITS[x % 2][y % 4]
                if not masks[index] & bit:
                    masks[index] |= bit
                    hot[index] = 1
            self.reveal.append((tuple(masks), hot))
        self.masks = tuple(masks)

    def map_dot(self, x: float, y: float) -> tuple[int, int]:
        return round((x - 139) * self.scale + self.ox), round((y - 657) * self.scale + self.oy)

    def cells(self, frame: int) -> tuple[tuple[int, ...], dict[int, int]]:
        frame = max(0, frame)
        if frame < REVEAL_FRAMES:
            # Keep the same reveal duration when a smaller terminal has fewer bins.
            index = min(len(self.reveal) - 1, frame * len(self.reveal) // REVEAL_FRAMES)
            return self.reveal[index]
        if frame >= REVEAL_FRAMES + ORBIT_FRAMES:
            return self.masks, {}
        travel = (frame - REVEAL_FRAMES) * ANGLE_STEP
        orbit = min(5, int(travel / math.tau))
        angle = travel % math.tau
        a, b = self.orbits[orbit]
        hot = {}
        # Tail first, head last: identical cell coloring to the PowerShell script.
        for i in range(11, -1, -1):
            tail_angle = angle - i * .055
            if tail_angle < 0:
                continue
            x, y = self.map_dot(2340 - a + a * math.cos(tail_angle), 1260 + b * math.sin(tail_angle))
            if 0 <= x < self.cols * 2 and 0 <= y < self.rows * 4:
                hot[y // 4 * self.cols + x // 2] = 0 if i == 0 else 1 if i < 3 else 2 if i < 6 else 3 if i < 9 else 4
        return self.masks, hot

    def render(self, frame: int, monochrome: bool = False) -> Text:
        masks, hot = self.cells(frame)
        chars = [chr(0x2800 + mask) if mask else " " for mask in masks]
        if monochrome:
            for index, color in hot.items():
                chars[index] = "@o*.."[color]
        text = Text("\n".join("".join(chars[r * self.cols:(r + 1) * self.cols]) for r in range(self.rows)),
                    style=BASE_COLOR, no_wrap=True, overflow="crop")
        for index, color in hot.items():
            offset = index + index // self.cols
            text.stylize(COLORS[color], offset, offset + 1)
        return text


@lru_cache(maxsize=4)
def animation(cols: int, rows: int) -> OrbitalAnimation:
    return OrbitalAnimation(cols, rows)
