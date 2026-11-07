import json
import shutil
import sys
from pathlib import Path

import pytest

from rvdlab.__main__ import main
from rvdlab.project import EXE, find_build, find_root, is_bundled, load_project, read_version

from conftest import ROOT


def bundle(tmp_path: Path, with_binary: bool = True) -> Path:
    """The wheel's rvdlab/runtime layout, assembled from the checkout."""
    runtime = tmp_path / "site-packages" / "rvdlab" / "runtime"
    shutil.copytree(ROOT / "configs", runtime / "configs")
    shutil.copytree(ROOT / "policies", runtime / "policies")
    (runtime / "runtime.json").write_text(json.dumps({"project": "rvd_railbench", "version": "9.8.7",
                                                      "simulation_only": True, "build_type": "Release"}))
    built = find_build(ROOT)
    if with_binary and built.rail_main is not None:
        (runtime / "bin").mkdir()
        shutil.copy2(built.rail_main, runtime / "bin" / f"rail_main{EXE}")
    return runtime


def test_bundled_runtime_is_a_root_with_its_own_version(tmp_path):
    runtime = bundle(tmp_path, with_binary=False)
    assert is_bundled(runtime) and not is_bundled(ROOT)
    assert find_root(runtime) == runtime
    assert read_version(runtime) == "9.8.7"


def test_bundled_runtime_keeps_user_data_outside_the_package(tmp_path, monkeypatch):
    runtime = bundle(tmp_path, with_binary=False)
    monkeypatch.setenv("RVDLAB_HOME", str(tmp_path / "home"))
    monkeypatch.delenv("RVDLAB_PROFILES", raising=False)
    monkeypatch.delenv("RVDLAB_EXPERIMENTS", raising=False)
    project = load_project(runtime)
    assert project.profiles_dir == (tmp_path / "home" / "profiles").resolve()
    assert project.experiments_dir == (tmp_path / "home" / "experiments").resolve()
    assert [f.name for f in project.profile.files] == ["rail.ini", "pfcr_dqp.ini", "simulation.ini"]
    assert project.policy.verified


@pytest.mark.skipif(find_build(ROOT).rail_main is None, reason="rail_main is not built")
def test_bundled_binary_is_found_and_reports_simulation_only(tmp_path):
    runtime = bundle(tmp_path)
    build = find_build(runtime)
    assert build.rail_main == runtime / "bin" / f"rail_main{EXE}"
    assert build.simulation_only is True


def test_version_flag(capsys, profiles):
    assert main(["--root", str(ROOT), "--profiles", str(profiles), "--version"]) == 0
    assert f"runtime {read_version(ROOT)}" in capsys.readouterr().out


@pytest.mark.skipif(find_build(ROOT).rail_main is None, reason="rail_main is not built")
def test_check_flag_runs_the_simulator(capsys, profiles):
    assert main(["--root", str(ROOT), "--profiles", str(profiles), "--check"]) == 0
    out = capsys.readouterr().out
    assert "Simulator" in out and "● Ready" in out and "Connected" not in out


def test_check_flag_fails_without_a_simulator(tmp_path, capsys, monkeypatch):
    runtime = bundle(tmp_path, with_binary=False)
    monkeypatch.setenv("RVDLAB_HOME", str(tmp_path / "home"))
    assert main(["--root", str(runtime), "--check"]) == 1
    assert "Not built" in capsys.readouterr().out


def test_interface_version_follows_the_project_version():
    from rvdlab import __version__

    assert __version__ == read_version(ROOT)
