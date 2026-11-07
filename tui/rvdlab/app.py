from __future__ import annotations

from pathlib import Path

from textual.app import App
from textual.binding import Binding
from textual.theme import Theme

from .config import Vec3
from .project import load_project
from .screens import (HomeScreen, ManagerScreen, MotionScreen, OverviewScreen, RendezvousScreen, SensorsScreen,
                      SetupScreen)

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


class RvdLabApp(App):
    CSS_PATH = "rvdlab.tcss"
    TITLE = "RVDLAB"
    ENABLE_COMMAND_PALETTE = False
    BINDINGS = [Binding("ctrl+q", "quit", "Quit", show=False, priority=True)]

    def __init__(self, root: Path | None = None, configs: list[Path] | None = None,
                 experiments: Path | None = None, profiles: Path | None = None) -> None:
        super().__init__()
        self._sources = (root, configs, experiments, profiles)
        self.project = load_project(root, configs, experiments, profiles)
        self.start_override: Vec3 | None = None

    def reload_project(self) -> None:
        self.project = load_project(*self._sources)

    def get_default_screen(self) -> HomeScreen:
        return HomeScreen()

    def on_mount(self) -> None:
        self.register_theme(MONOCHROME)
        self.theme = MONOCHROME.name
        for name, screen in (
            ("overview", OverviewScreen()),
            ("motion", MotionScreen()),
            ("rendezvous", RendezvousScreen()),
            ("sensors", SensorsScreen()),
            ("manager", ManagerScreen()),
            ("setup", SetupScreen()),
        ):
            self.install_screen(screen, name)
        if self.project.rail_profile is None and not self.project.explicit_configs:
            self.push_screen("setup")  # first start: describe the rail before anything else
