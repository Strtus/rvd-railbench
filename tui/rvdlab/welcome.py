"""Braille orbital welcome, ported from the supplied rvdlab_braille_anim.ps1.

Preserves its logo geometry, reveal order and ANSI palette. Comet motion uses
Kepler's equation on each stylized ellipse (the logo has no shared focus).
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from functools import lru_cache

from rich.text import Text

from .logo import LOGO
from .project import user_data_dir

COLS, ROWS, FPS = 108, 27, 24
REVEAL_DURATION = 3.0
REVEAL_FRAMES = round(REVEAL_DURATION * FPS)
ORBIT_DURATION = 2.5
DURATION = REVEAL_DURATION + ORBIT_DURATION
MEAN_ANOMALY_STEP = 0.105
ORBIT_AXIS_RATIO = .545
ECCENTRICITY = math.sqrt(1 - ORBIT_AXIS_RATIO**2)
SHRINK_DURATION = 1.6
TYPE_START = DURATION + SHRINK_DURATION
# Matches the public site's 2.7 s, six-letter wordmark typing sequence.
LETTER_TIMES = (.9, 1.5, 1.95, 2.28, 2.52, 2.7)
CURSOR_DURATION = 3.6
BASE_COLOR = "#005faf"
COLORS = ("#ffffff", "#00d7ff", "#00afff", "#0087ff", "#005fff")
BITS = ((1, 2, 4, 64), (8, 16, 32, 128))


def eccentric_anomaly(mean_anomaly: float) -> float:
    """Solve E - e sin(E) = M for equal swept areas in equal times."""
    mean_anomaly %= math.tau
    angle = mean_anomaly
    for _ in range(12):
        correction = (angle - ECCENTRICITY * math.sin(angle) - mean_anomaly) / (1 - ECCENTRICITY * math.cos(angle))
        angle -= correction
        if abs(correction) < 1e-12:
            break
    return angle


def _comet_phase(frame: int) -> tuple[int, float]:
    travel = max(0, frame - REVEAL_FRAMES) * MEAN_ANOMALY_STEP
    return int(travel / math.tau) % 6, travel % math.tau


def comet_points(frame: int):
    """Share one continuous clock across the opening, zoom and wordmark."""
    orbit, mean_anomaly = _comet_phase(frame)
    a = 1100.5 - 162 * orbit
    b = a * ORBIT_AXIS_RATIO
    # Tail samples earlier times, so its length follows the comet's speed.
    for i in range(11, -1, -1):
        tail_mean = mean_anomaly - i * .055
        if tail_mean < 0:
            continue
        angle = eccentric_anomaly(tail_mean)
        color = 0 if i == 0 else 1 if i < 3 else 2 if i < 6 else 3 if i < 9 else 4
        yield 2340 - a + a * math.cos(angle), 1260 + b * math.sin(angle), color


def _comet_cells(frame: int, cols: int, rows: int, owners: Sequence[int],
                 project: Callable[[float, float], tuple[int, int]]) -> dict[int, int]:
    orbit, _ = _comet_phase(frame)
    hot = {}
    for px, py, color in comet_points(frame):
        x, y = project(px, py)
        if 0 <= x < cols * 2 and 0 <= y < rows * 4:
            index = y // 4 * cols + x // 2
            # A terminal colors the entire Braille cell. Leave shared cells at
            # their base color rather than lighting unrelated orbits or the hub.
            if owners[index] == 1 << orbit:
                hot[index] = color
    return hot


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
        self.orbits = [(1100.5 - 162 * k, (1100.5 - 162 * k) * ORBIT_AXIS_RATIO) for k in range(6)]
        dots: set[tuple[int, int]] = set()
        self.owners = [0] * (cols * rows)

        def put(x: float, y: float, owner: int) -> None:
            dx, dy = self.map_dot(x, y)
            if 0 <= dx < dw and 0 <= dy < dh:
                dots.add((dx, dy))
                self.owners[dy // 4 * cols + dx // 2] |= 1 << owner

        for owner, (a, b) in enumerate(self.orbits):
            steps = round(max(1400, a * 3))
            for i in range(steps + 1):
                angle = math.tau * i / steps
                put(2340 - a + a * math.cos(angle), 1260 + b * math.sin(angle), owner)
        for i in range(600):
            angle = math.tau * i / 600
            put(2307.5 + 32.5 * math.cos(angle), 1260 + 32.5 * math.sin(angle), 6)

        apex_x = (2340 - 139) * self.scale + self.ox
        apex_y = (1260 - 657) * self.scale + self.oy
        distances = {(x, y): math.hypot(x - apex_x, y - apex_y) for x, y in dots}
        maximum = max(1.0, *distances.values())
        bins: list[list[tuple[int, int]]] = [[] for _ in range(REVEAL_FRAMES)]
        for point, distance in distances.items():
            bins[min(REVEAL_FRAMES - 1, round((distance / maximum)**.92 * (REVEAL_FRAMES - 1)))].append(point)
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
        hot = _comet_cells(frame, self.cols, self.rows, self.owners, self.map_dot)
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


@lru_cache(maxsize=1)
def _outline() -> tuple[tuple[float, float, int], ...]:
    points = []
    for k in range(6):
        a = 1100.5 - 162 * k
        b = a * .545
        steps = round(max(1400, a * 3))
        for i in range(steps + 1):
            angle = math.tau * i / steps
            points.append((2340 - a + a * math.cos(angle), 1260 + b * math.sin(angle), k))
    for i in range(600):
        angle = math.tau * i / 600
        points.append((2307.5 + 32.5 * math.cos(angle), 1260 + 32.5 * math.sin(angle), 6))
    return tuple(points)


def _project(x: float, y: float, scale: float, cx: float, cy: float) -> tuple[int, int]:
    return round((x - 1239.5) * scale + cx), round((y - 1260) * scale + cy)


@lru_cache(maxsize=64)
def _projected_geometry(cols: int, rows: int, scale: float, cx: float, cy: float) -> tuple[tuple[int, ...], tuple[int, ...]]:
    masks = [0] * (cols * rows)
    owners = [0] * (cols * rows)
    for x, y, owner in _outline():
        dx, dy = _project(x, y, scale, cx, cy)
        if 0 <= dx < cols * 2 and 0 <= dy < rows * 4:
            index = dy // 4 * cols + dx // 2
            masks[index] |= BITS[dx % 2][dy % 4]
            owners[index] |= 1 << owner
    return tuple(masks), tuple(owners)


class WelcomeAnimation:
    """A text-free opening, a continuous zoom, then the six-line wordmark.

    Geometry is projected on the same 2 x 4 Braille dot grid throughout the
    zoom. The final logo has no top/bottom padding: its occupied rows are the
    exact same rows as the wordmark, including the last shadow row.
    """

    def __init__(self, cols: int, rows: int) -> None:
        self.cols, self.rows = cols, rows
        self.wordmark = LOGO.splitlines()
        self.stops = (8, 17, 25, 33, 41, 49)
        self.gap = 4
        if cols < 77 or rows < 6:
            # Keep the logo beside the text, rather than wrapping or clipping it.
            self.wordmark = ["█▀█ █ █ █▀▄ █   ▄▀▄ █▀▄", "█▀▄ █ █ █ █ █   █▀█ █▀▄", "▀ ▀  ▀  ▀▀  ▀▀▀ ▀ ▀ ▀▀ "]
            self.stops = (3, 7, 11, 15, 19, 23)
            self.gap = 3
        if cols < 40 or rows < 3:
            self.wordmark = ["RVDLAB"]
            self.stops = (1, 2, 3, 4, 5, 6)
            self.gap = 2
        self.logo_height = len(self.wordmark)
        self.target_scale = (self.logo_height * 4 - 1) / (2 * 1100.5 * .545)
        dot_width = 2201 * self.target_scale
        self.logo_width = math.ceil((round(dot_width) + 1) / 2)
        self.wordmark_width = max(map(len, self.wordmark))
        total_width = self.logo_width + self.gap + self.wordmark_width + 2
        self.logo_x = max(0, (cols - total_width) // 2)
        self.logo_y = max(0, (rows - self.logo_height) // 2)
        self.title_x = self.logo_x + self.logo_width + self.gap
        self.target_cx = self.logo_x * 2 + dot_width / 2
        self.target_cy = self.logo_y * 4 + (self.logo_height * 4 - 1) / 2
        self.full_cols, self.full_rows = min(COLS, cols), min(ROWS, rows)
        self.full_x = (cols - self.full_cols) // 2
        self.full_y = (rows - self.full_rows) // 2

    def render(self, elapsed: float, monochrome: bool = False) -> Text:
        frame = max(0, int(elapsed * FPS))
        full = animation(self.full_cols, self.full_rows)
        hot = {}
        if elapsed < DURATION:
            small, source_hot = full.cells(frame)
            masks = [0] * (self.cols * self.rows)
            for index, mask in enumerate(small):
                target = (index // self.full_cols + self.full_y) * self.cols + index % self.full_cols + self.full_x
                masks[target] = mask
                if index in source_hot:
                    hot[target] = source_hot[index]
        else:
            progress = min(1.0, (elapsed - DURATION) / SHRINK_DURATION)
            progress = max(0.0, progress)
            eased = progress * progress * (3 - 2 * progress)
            scale = full.scale + (self.target_scale - full.scale) * eased
            start_cx = self.full_x * 2 + self.full_cols
            start_cy = self.full_y * 4 + self.full_rows * 2
            cx = start_cx + (self.target_cx - start_cx) * eased
            cy = start_cy + (self.target_cy - start_cy) * eased
            masks, owners = _projected_geometry(self.cols, self.rows, scale, cx, cy)
            hot = _comet_cells(frame, self.cols, self.rows, owners,
                               lambda x, y: _project(x, y, scale, cx, cy))
        chars = [chr(0x2800 + mask) if mask else " " for mask in masks]
        if monochrome:
            for index, color in hot.items():
                chars[index] = "@o*.."[color]
        white = []
        if elapsed >= TYPE_START:
            typing_time = elapsed - TYPE_START
            letters = sum(typing_time >= moment for moment in LETTER_TIMES)
            visible = self.stops[letters - 1] if letters else 0
            cursor = typing_time < CURSOR_DURATION and int(typing_time / .3) % 2 == 0
            for r, line in enumerate(self.wordmark):
                y = self.logo_y + r
                for c, char in enumerate(line[:visible]):
                    x = self.title_x + c
                    if x < self.cols and y < self.rows:
                        index = y * self.cols + x
                        chars[index] = char
                        white.append(index)
                x = self.title_x + visible + (1 if visible else 0)
                if cursor and x < self.cols and y < self.rows:
                    index = y * self.cols + x
                    chars[index] = "█"
                    white.append(index)
        text = Text("\n".join("".join(chars[r * self.cols:(r + 1) * self.cols]) for r in range(self.rows)),
                    style=BASE_COLOR, no_wrap=True, overflow="crop")
        for index, color in hot.items():
            offset = index + index // self.cols
            text.stylize(COLORS[color], offset, offset + 1)
        for index in white:
            offset = index + index // self.cols
            text.stylize("bold #ededed", offset, offset + 1)
        return text


@lru_cache(maxsize=4)
def welcome_animation(cols: int, rows: int) -> WelcomeAnimation:
    return WelcomeAnimation(cols, rows)
