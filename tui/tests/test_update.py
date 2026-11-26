import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from rvdlab import update
from rvdlab.__main__ import main


@pytest.fixture
def release(monkeypatch):
    def install(site_packages):
        dist = SimpleNamespace(version="0.7.5", locate_file=lambda name: site_packages / name)
        monkeypatch.setattr(update.metadata, "distribution", Mock(return_value=dist))
        monkeypatch.setattr(update, "__file__", str(site_packages / "rvdlab" / "update.py"))

    return install


@pytest.mark.parametrize(("args", "text"), [
    (["--help"], "Use 'rvdlab update'"),
    (["update", "--help"], "usage: rvdlab update"),
])
def test_update_help(args, text, capsys, monkeypatch):
    run = Mock()
    monkeypatch.setattr(update.subprocess, "run", run)

    with pytest.raises(SystemExit) as exit_info:
        main(args)

    assert exit_info.value.code == 0
    assert text in capsys.readouterr().out
    run.assert_not_called()


@pytest.mark.parametrize(("in_user_site", "user_site_enabled", "flags"), [
    (True, True, ["--user"]),
    (False, True, []),
    (True, False, []),
])
def test_update_uses_current_python_and_returns_pip_status(
        in_user_site, user_site_enabled, flags, release, tmp_path, monkeypatch):
    user_site = tmp_path / "user-site"
    release(user_site if in_user_site else tmp_path / "site-packages")
    monkeypatch.setattr(update.site, "ENABLE_USER_SITE", user_site_enabled)
    monkeypatch.setattr(update.site, "getusersitepackages", lambda: str(user_site))
    monkeypatch.setattr(update.util, "find_spec", Mock(return_value=object()))
    run = Mock(return_value=SimpleNamespace(returncode=17))
    monkeypatch.setattr(update.subprocess, "run", run)

    assert main(["update"]) == 17
    run.assert_called_once_with(
        [sys.executable, "-m", "pip", "install", "--upgrade", *flags, "rvd-railbench"], check=False,
    )


@pytest.mark.parametrize("wheel_installed", [False, True])
def test_source_checkout_does_not_install_package(wheel_installed, release, tmp_path, capsys, monkeypatch):
    if wheel_installed:
        release(tmp_path / "site-packages")
    else:
        monkeypatch.setattr(update.metadata, "distribution", Mock(side_effect=update.metadata.PackageNotFoundError))
    monkeypatch.setattr(update, "__file__", str(tmp_path / "checkout" / "tui" / "rvdlab" / "update.py"))
    run = Mock()
    monkeypatch.setattr(update.subprocess, "run", run)

    assert main(["update"]) == 1
    assert "Update source checkouts through Git" in capsys.readouterr().err
    run.assert_not_called()


def test_missing_pip_does_not_install_package(release, tmp_path, capsys, monkeypatch):
    release(tmp_path / "site-packages")
    monkeypatch.setattr(update.util, "find_spec", Mock(return_value=None))
    run = Mock()
    monkeypatch.setattr(update.subprocess, "run", run)

    assert main(["update"]) == 1
    assert "pip is unavailable" in capsys.readouterr().err
    run.assert_not_called()


def test_windows_launcher_defers_pip_until_after_exit(release, tmp_path, monkeypatch):
    release(tmp_path / "site-packages")
    launcher = tmp_path / "Scripts" / "rvdlab.exe"
    monkeypatch.setattr(update.sys, "platform", "win32")
    monkeypatch.setattr(update.sys, "orig_argv", [sys.executable, str(launcher), "update"])
    monkeypatch.setattr(update.site, "ENABLE_USER_SITE", False)
    monkeypatch.setattr(update.util, "find_spec", Mock(return_value=object()))
    handoff = Mock(return_value=0)
    run = Mock()
    monkeypatch.setattr(update, "_update_after_windows_launcher", handoff)
    monkeypatch.setattr(update.subprocess, "run", run)

    assert main(["update"]) == 0
    handoff.assert_called_once_with(
        launcher.resolve(), [sys.executable, "-m", "pip", "install", "--upgrade", "rvd-railbench"],
    )
    run.assert_not_called()
