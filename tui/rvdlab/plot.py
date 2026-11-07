"""Monochrome braille line plot."""

from __future__ import annotations

from collections.abc import Callable, Sequence

from rich.text import Text
from textual.widget import Widget

_DOTS = ((0x01, 0x08), (0x02, 0x10), (0x04, 0x20), (0x40, 0x80))
AXIS = "#5f5f5f"
LABEL = "#8a8a8a"
TRACE = "#d0d0d0"
MARK = "bold #ffffff"

Marker = tuple[float, float, str]


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
                 y_name: str = "") -> Text:
    """Polyline of (xs, ys) in a width x height cell box, axes and labels included."""
    all_x = [*xs, *(m[0] for m in markers)]
    all_y = [*ys, *(m[1] for m in markers)]
    if not all_x:
        return Text("no data", style=LABEL)
    x0, x1 = _span(min(all_x), max(all_x))
    y0, y1 = _span(min(all_y), max(all_y))
    top, bottom = _label(y1), _label(y0)
    gutter = max(len(top), len(bottom), len(y_name)) + 1
    cols, rows = width - gutter - 1, height - 2
    if cols < 4 or rows < 2:
        return Text("")
    w, h = cols * 2, rows * 4
    cells = [[0] * cols for _ in range(rows)]

    def px(x: float) -> int:
        f = (x - x0) / (x1 - x0)
        return min(w - 1, max(0, round((1 - f if invert_x else f) * (w - 1))))

    def py(y: float) -> int:
        return min(h - 1, max(0, round((y1 - y) / (y1 - y0) * (h - 1))))

    def dot(x: int, y: int) -> None:
        cells[y // 4][x // 2] |= _DOTS[y % 4][x % 2]

    previous = None
    for x, y in zip(xs, ys):
        current = (px(x), py(y))
        _line(previous or current, current, dot)
        previous = current
    marks = {(py(y) // 4, px(x) // 2): glyph for x, y, glyph in markers}

    text = Text(no_wrap=True, overflow="crop")
    for r in range(rows):
        label = top if r == 0 else bottom if r == rows - 1 else y_name if r == rows // 2 else ""
        text.append(label.rjust(gutter - 1) + " ", style=LABEL)
        text.append("┤" if r in (0, rows - 1) else "│", style=AXIS)
        run = []
        for c in range(cols):
            if (r, c) in marks:
                text.append("".join(run), style=TRACE)
                text.append(marks[(r, c)], style=MARK)
                run = []
            else:
                run.append(chr(0x2800 + cells[r][c]) if cells[r][c] else " ")
        text.append("".join(run) + "\n", style=TRACE)
    text.append(" " * gutter + "└" + "─" * cols + "\n", style=AXIS)
    left, right = (_label(x1), _label(x0)) if invert_x else (_label(x0), _label(x1))
    middle = x_name.center(max(0, cols - len(left) - len(right)))
    text.append(" " * (gutter + 1) + (left + middle + right)[:cols], style=LABEL)
    return text


class PlotView(Widget):
    DEFAULT_CSS = "PlotView { height: 1fr; }"

    def __init__(self, caption: str, *, invert_x: bool = False, x_name: str = "", y_name: str = "",
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self._caption = caption
        self._invert_x = invert_x
        self._x_name = x_name
        self._y_name = y_name
        self._xs: list[float] = []
        self._ys: list[float] = []
        self._markers: list[Marker] = []

    def plot(self, xs: Sequence[float], ys: Sequence[float], markers: Sequence[Marker] = ()) -> None:
        self._xs, self._ys, self._markers = list(xs), list(ys), list(markers)
        self.refresh()

    def render(self) -> Text:
        head = Text(self._caption, style="bold #d0d0d0", no_wrap=True, overflow="crop")
        if self.size.height < 4 or not self._xs:
            return Text("\n").join([head, Text("no data", style=LABEL)]) if self.size.height > 1 else head
        body = braille_plot(self._xs, self._ys, self.size.width, self.size.height - 1, invert_x=self._invert_x,
                            markers=self._markers, x_name=self._x_name, y_name=self._y_name)
        return Text("\n").join([head, body])
