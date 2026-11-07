from pathlib import Path

import pytest

from rvdlab.config import Profile, apply_file
from rvdlab.profiles import RailProfile, save, set_active
from rvdlab.project import find_root

ROOT = find_root(Path(__file__).resolve().parent)


@pytest.fixture
def profiles(tmp_path: Path) -> Path:
    """Profile directory with the checked-in example rail saved and active."""
    example = Profile()
    for name in ("rail.ini", "simulation.ini"):
        apply_file(example, ROOT / "configs" / name)
    directory = tmp_path / "profiles"
    save(directory, RailProfile("example-rail", example.rail, example.simulation.start_hill_m))
    set_active(directory, "example-rail")
    return directory
