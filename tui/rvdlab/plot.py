"""Braille line plot with highlighted event positions."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass

from rich.text import Text
from textual.widget import Widget

_DOTS = ((0x01, 0x08), (0x02, 0x10), (0x04, 0x20), (0x40, 0x80))
AXIS = "#5f5f5f"
LABEL = "#8a8a8a"
TRACE = "#d0d0d0"
MARK = "bold #ffffff"
HIGHLIGHT = "#ffaf00"
HIGHLIGHT_GLYPH = "×"
NOMINAL = "#cf87ff"
RAW = "#5fd7ff"
FILTERED = "#5fdf87"

Marker = tuple[float, float, str]


@dataclass
class Trace:
    xs: Sequence[float]
    ys: Sequence[float]
    style: str
    dashed: bool = False


def _label(value: float) -> str:
    size = abs(value)
    if size >= 100:
        return f"{value:.0f}"
    if size >= 10:
        return f"{value:.1f}"
    if size >= 1:
        return f"{value:.2f}"
    return "0" if size < 5e-4 else f"{value:.3f}"


def _span(lo: float, hi: float) -> tuple[float, float]:
    return (lo - 0.5, hi + 0.5) if hi - lo < 1e-9 else (lo, hi)


def _line(a: tuple[int, int], b: tuple[int, int], dot: Callable[[int, int], None]) -> None:
    (x, y), (xb, yb) = a, b
    dx, dy = abs(xb - x), -abs(yb - y)
    sx, sy = (1 if x < xb else -1), (1 if y < yb else -1)
    err = dx + dy
    while True:
        dot(x, y)
        if x == xb and y == yb:
            return
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy


def braille_plot(xs: Sequence[float], ys: Sequence[float], width: int, height: int, *,
                 invert_x: bool = False, markers: Sequence[Marker] = (), x_name: str = "",
                 y_name: str = "", highlights: Sequence[tuple[float, float]] = (),
                 overlays: Sequence[Trace] = (), label_width: int = 0) -> Text:
    """Polyline of (xs, ys) in a width x height cell box, axes and labels included."""
    all_x = [*xs, *(m[0] for m in markers), *(x for trace in overlays for x in trace.xs)]
    all_y = [*ys, *(m[1] for m in markers), *(y for trace in overlays for y in trace.ys)]
    if not all_x:
        return Text("no data", style=LABEL)
    x0, x1 = _span(min(all_x), max(all_x))
    y0, y1 = _span(min(all_y), max(all_y))
    top, bottom = _label(y1), _label(y0)
    gutter = max(len(top), len(bottom), len(y_name), label_width) + 1
    cols, rows = width - gutter - 1, height - 2
    if cols < 4 or rows < 2:
        return Text("")
    w, h = cols * 2, rows * 4
    cells = [[0] * cols for _ in range(rows)]
    overlay_cells: dict[tuple[int, int], tuple[int, str]] = {}

    def px(x: float) -> int:
        f = (x - x0) / (x1 - x0)
        return min(w - 1, max(0, round((1 - f if invert_x else f) * (w - 1))))

    def py(y: float) -> int:
        return min(h - 1, max(0, round((y1 - y) / (y1 - y0) * (h - 1))))

    def dot(x: int, y: int) -> None:
        cells[y // 4][x // 2] |= _DOTS[y % 4][x % 2]

    for trace in overlays:
        def overlay_dot(x: int, y: int) -> None:
            if trace.dashed and (x + y) % 6 >= 3:
                return
            key = (y // 4, x // 2)
            bits, _ = overlay_cells.get(key, (0, trace.style))
            overlay_cells[key] = (bits | _DOTS[y % 4][x % 2], trace.style)

        previous = None
        for x, y in zip(trace.xs, trace.ys):
            current = (px(x), py(y))
            _line(previous or current, current, overlay_dot)
            previous = current

    previous = None
    for x, y in zip(xs, ys):
        current = (px(x), py(y))
        _line(previous or current, current, dot)
        previous = current
    marks = {(py(y) // 4, px(x) // 2): glyph for x, y, glyph in markers}
    highlighted = {(py(y) // 4, px(x) // 2) for x, y in highlights}

    text = Text(no_wrap=True, overflow="crop")
    for r in range(rows):
        label = top if r == 0 else bottom if r == rows - 1 else y_name if r == rows // 2 else ""
        text.append(label.rjust(gutter - 1) + " ", style=LABEL)
        text.append("┤" if r in (0, rows - 1) else "│", style=AXIS)
        run = []
        for c in range(cols):
            if (r, c) in marks or (r, c) in highlighted:
                text.append("".join(run), style=TRACE)
                # Keep the current/target glyph when an event shares its cell.
                text.append(marks.get((r, c), HIGHLIGHT_GLYPH),
                            style=HIGHLIGHT if (r, c) in highlighted else MARK)
                run = []
            elif not cells[r][c] and (r, c) in overlay_cells:
                text.append("".join(run), style=TRACE)
                bits, style = overlay_cells[(r, c)]
                text.append(chr(0x2800 + bits), style=style)
                run = []
            else:
                run.append(chr(0x2800 + cells[r][c]) if cells[r][c] else " ")
        text.append("".join(run) + "\n", style=TRACE)
    text.append(" " * gutter + "└" + "─" * cols + "\n", style=AXIS)
    left, right = (_label(x1), _label(x0)) if invert_x else (_label(x0), _label(x1))
    labels = list(" " * cols)
    labels[:len(left)] = left
    labels[cols - len(right):] = right
    center = (cols - len(x_name)) // 2
    if center > len(left) and center + len(x_name) < cols - len(right):
        labels[center:center + len(x_name)] = x_name
    text.append(" " * (gutter + 1) + "".join(labels)[:cols], style=LABEL)
    return text


class PlotView(Widget):
    DEFAULT_CSS = "PlotView { height: 1fr; }"

    def __init__(self, caption: str, *, invert_x: bool = False, x_name: str = "", y_name: str = "",
                 highlight_label: str = "",
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self._caption = caption
        self._invert_x = invert_x
        self._x_name = x_name
        self._y_name = y_name
        self._highlight_label = highlight_label
        self._xs: list[float] = []
        self._ys: list[float] = []
        self._markers: list[Marker] = []
        self._highlights: list[tuple[float, float]] = []
        self._overlays: list[Trace] = []
        self._label_width = 0

    @property
    def axis_label_width(self) -> int:
        values = [*self._ys, *(m[1] for m in self._markers),
                  *(y for trace in self._overlays for y in trace.ys)]
        limits = _span(min(values), max(values)) if values else (0, 0)
        return max(len(self._y_name), *(len(_label(value)) for value in limits))

    def align_axis_labels(self, width: int) -> None:
        self._label_width = width
        self.refresh()

    def plot(self, xs: Sequence[float], ys: Sequence[float], markers: Sequence[Marker] = (), *,
             highlights: Sequence[tuple[float, float]] = (), overlays: Sequence[Trace] = ()) -> None:
        self._xs, self._ys, self._markers = list(xs), list(ys), list(markers)
        self._highlights = list(highlights)
        self._overlays = list(overlays)
        self.refresh()

    def render(self) -> Text:
        head = Text(self._caption, style="bold #d0d0d0", no_wrap=True, overflow="crop")
        if self._highlight_label:
            head.append(f"  {HIGHLIGHT_GLYPH} {self._highlight_label}", style=HIGHLIGHT)
        if self.size.height < 4 or (not self._xs and not self._overlays):
            return Text("\n").join([head, Text("no data", style=LABEL)]) if self.size.height > 1 else head
        body = braille_plot(self._xs, self._ys, self.size.width, self.size.height - 1, invert_x=self._invert_x,
                            markers=self._markers, x_name=self._x_name, y_name=self._y_name,
                            highlights=self._highlights, overlays=self._overlays, label_width=self._label_width)
        return Text("\n").join([head, body])
