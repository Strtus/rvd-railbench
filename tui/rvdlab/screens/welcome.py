from __future__ import annotations

from time import monotonic

from rich.text import Text
from textual import events
from textual.app import ComposeResult
from textual.reactive import reactive
from textual.screen import Screen
from textual.timer import Timer
from textual.widget import Widget

from ..welcome import FPS, welcome_animation


class OrbitalArt(Widget):
    elapsed = reactive(0.0)

    def render(self) -> Text:
        cols, rows = self.content_size
        if cols < 16 or rows < 4:
            return Text("")
        return welcome_animation(cols, rows).render(self.elapsed, monochrome=self.app.console.no_color)


class WelcomeScreen(Screen[None]):
    DEFAULT_CSS = """
    WelcomeScreen {
        align: center middle;
        background: #0b0b0b;
    }
    #welcome-art {
        width: 1fr;
        height: 1fr;
    }
    """

    def __init__(self) -> None:
        super().__init__()
        self._finished = False
        self._timer: Timer | None = None
        self._started = 0.0

    def compose(self) -> ComposeResult:
        yield OrbitalArt(id="welcome-art")

    def on_mount(self) -> None:
        self._started = monotonic()
        self._timer = self.set_interval(1 / FPS, self._tick)

    def _tick(self) -> None:
        if self._finished:
            return
        self.query_one(OrbitalArt).elapsed = monotonic() - self._started

    def on_key(self, event: events.Key) -> None:
        event.stop()
        event.prevent_default()
        self.finish()

    def finish(self) -> None:
        if self._finished:
            return
        self._finished = True
        if self._timer is not None:
            self._timer.stop()
        self.dismiss()

    def on_unmount(self) -> None:
        if self._timer is not None:
            self._timer.stop()
