from __future__ import annotations

from pathlib import Path
from typing import Literal

from textual.app import App
from textual.binding import Binding
from textual.theme import Theme
from textual.timer import Timer

from .config import Vec3
from .project import load_project
from .remote import connection_path
from .screens import (HomeScreen, ManagerScreen, MotionScreen, OverviewScreen, RendezvousScreen, SensorsScreen,
                      SetupScreen)
from .screens.welcome import WelcomeScreen
from .welcome import mark_welcome_seen, welcome_needed

MONOCHROME = Theme(
    name="rvd-mono",
    primary="#cfcfcf",
    secondary="#9e9e9e",
    accent="#ffffff",
    foreground="#d6d6d6",
    background="#0b0b0b",
    surface="#121212",
    panel="#1c1c1c",
    boost="#ffffff",
    warning="#e6e6e6",
    error="#ffffff",
    success="#cfcfcf",
    dark=True,
)

RailControlState = Literal["idle", "running", "intervening", "fault"]


class RvdLabApp(App):
    CSS_PATH = "rvdlab.tcss"
    TITLE = "RVDLAB"
    ENABLE_COMMAND_PALETTE = False
    BINDINGS = [Binding("ctrl+q", "quit", "Quit", show=False, priority=True)]

    def __init__(self, root: Path | None = None, configs: list[Path] | None = None,
                 experiments: Path | None = None, profiles: Path | None = None, node: Path | None = None,
                 welcome: bool | None = None) -> None:
        super().__init__()
        self._sources = (root, configs, experiments, profiles)
        self.project = load_project(root, configs, experiments, profiles)
        self.start_override: Vec3 | None = None
        self.node_path = connection_path(node)
        self._welcome = welcome
        self.rail_control_state: RailControlState = "idle"
        self._rail_control_timer: Timer | None = None

    def set_rail_control_state(self, state: RailControlState) -> None:
        """Show steady running/CBF colors; only faults pulse, on every screen."""
        if state == self.rail_control_state:
            return
        if self._rail_control_timer is not None:
            self._rail_control_timer.stop()
            self._rail_control_timer = None
        self.rail_control_state = state
        self.set_class(state != "idle", "rail-control-active", update=False)
        self.set_class(state == "intervening", "rail-control-intervening", update=False)
        self.set_class(state == "fault", "rail-control-fault", "rail-control-bright", update=False)
        self.update_node_styles(animate=False)
        if state == "fault":
            self._rail_control_timer = self.set_interval(0.5, self._pulse_rail_control)

    def _pulse_rail_control(self) -> None:
        if self._rail_control_timer is not None:
            self.toggle_class("rail-control-bright")

    def reload_project(self) -> None:
        self.project = load_project(*self._sources)

    def export_screenshot(self, *, title: str | None = None, simplify: bool = False) -> str:
        svg = super().export_screenshot(title=title, simplify=simplify)
        # Apply the font to text nodes as well as groups, including symbol-led labels.
        # The terminal itself uses the font selected in the user's terminal emulator.
        fonts = ('text { font-family: "Cascadia Mono", "DejaVu Sans Mono", '
                 '"Lucida Console", "Segoe UI Symbol", monospace; }')
        return svg.replace("</style>", fonts + "\n</style>", 1)

    def get_default_screen(self) -> HomeScreen:
        return HomeScreen()

    def on_mount(self) -> None:
        self.register_theme(MONOCHROME)
        self.theme = MONOCHROME.name
        # Cached screens also need the current warning styles when popped back into view.
        self.screen_change_signal.subscribe(
            self, lambda screen: screen.update_node_styles(animate=False), immediate=True)
        for name, screen in (
            ("overview", OverviewScreen()),
            ("motion", MotionScreen()),
            ("rendezvous", RendezvousScreen()),
            ("sensors", SensorsScreen()),
            ("manager", ManagerScreen()),
            ("setup", SetupScreen()),
        ):
            self.install_screen(screen, name)
        show_welcome = self._welcome if self._welcome is not None else not self.is_headless and welcome_needed()
        if show_welcome:
            self.push_screen(WelcomeScreen(), self._welcome_finished)
        else:
            self._open_start_screen()

    def _welcome_finished(self, _result: None) -> None:
        mark_welcome_seen()
        self._open_start_screen()

    def _open_start_screen(self) -> None:
        if self.project.rail_profile is None and not self.project.explicit_configs:
            self.push_screen("setup")  # first start: describe the rail before anything else
