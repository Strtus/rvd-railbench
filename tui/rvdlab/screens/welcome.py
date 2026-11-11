from __future__ import annotations

from time import monotonic

from rich.text import Text
from textual import events
from textual.app import ComposeResult
from textual.containers import Vertical
from textual.reactive import reactive
from textual.screen import Screen
from textual.timer import Timer
from textual.widget import Widget
from textual.widgets import Static

from ..welcome import COLS, DURATION, FPS, ROWS, animation


class OrbitalArt(Widget):
    frame = reactive(0)

    def render(self) -> Text:
        cols, rows = min(COLS, self.content_size.width), min(ROWS, self.content_size.height)
        if cols < 16 or rows < 4:
            return Text("")
        return animation(cols, rows).render(self.frame, monochrome=self.app.console.no_color)


class WelcomeScreen(Screen[None]):
    DEFAULT_CSS = """
    WelcomeScreen {
        align: center middle;
        background: #0b0b0b;
    }
    WelcomeScreen > Vertical {
        width: 112;
        max-width: 100%;
        height: 33;
        max-height: 100%;
        padding: 1 2;
        align: center middle;
    }
    #welcome-title {
        height: 1;
        text-align: center;
        text-style: bold;
        color: #ededed;
    }
    #welcome-art {
        width: 1fr;
        height: 1fr;
        max-width: 108;
        max-height: 27;
        margin: 1 0;
    }
    #welcome-hint {
        height: 1;
        text-align: center;
        color: #8a8a8a;
    }
    """

    def __init__(self) -> None:
        super().__init__()
        self._finished = False
        self._timer: Timer | None = None
        self._started = 0.0

    def compose(self) -> ComposeResult:
        with Vertical():
            yield Static("R V D L A B", id="welcome-title")
            yield OrbitalArt(id="welcome-art")
            yield Static("Press any key to skip", id="welcome-hint")

    def on_mount(self) -> None:
        self._started = monotonic()
        self._timer = self.set_interval(1 / FPS, self._tick)

    def _tick(self) -> None:
        if self._finished:
            return
        elapsed = monotonic() - self._started
        if elapsed >= DURATION:
            self.finish()
        else:
            self.query_one(OrbitalArt).frame = int(elapsed * FPS)

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
