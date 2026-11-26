"""Upgrade a packaged RVDLAB installation with its Python environment's pip."""

from __future__ import annotations

import argparse
from importlib import metadata, util
import json
import os
from pathlib import Path
import site
import subprocess
import sys


PACKAGE = "rvd-railbench"

WINDOWS_UPDATE = r"""
$ErrorActionPreference = 'Stop'
$running = @(Get-Process -Name rvdlab -ErrorAction SilentlyContinue |
    Where-Object { $_.Path -eq $env:RVDLAB_UPDATE_LAUNCHER })
if ($running.Count) {
    $running | Wait-Process -Timeout 30
}

$command = ConvertFrom-Json $env:RVDLAB_UPDATE_COMMAND
$python = $command[0]
$pipArgs = $command[1..($command.Count - 1)]
& $python @pipArgs
$result = $LASTEXITCODE
if ($result -eq 0) {
    Write-Output 'RVDLAB update complete.'
} else {
    [Console]::Error.WriteLine("RVDLAB update failed (pip exit code $result).")
}
exit $result
"""


def _update_after_windows_launcher(launcher: Path, command: list[str]) -> int:
    powershell = Path(os.environ.get("SystemRoot", r"C:\Windows")) / "System32" / "WindowsPowerShell" / "v1.0"
    env = {**os.environ, "RVDLAB_UPDATE_LAUNCHER": str(launcher), "RVDLAB_UPDATE_COMMAND": json.dumps(command)}
    try:
        # The launcher stays alive until Python exits, so pip must run after it releases the .exe.
        subprocess.Popen([str(powershell / "powershell.exe"), "-NoLogo", "-NoProfile", "-NonInteractive",
                          "-Command", WINDOWS_UPDATE], env=env, stdin=subprocess.DEVNULL)
    except OSError as exc:
        print(f"Could not start the Windows updater: {exc}. Try python -m rvdlab update.", file=sys.stderr)
        return 1
    print("Waiting for the Windows launcher to exit. Keep this terminal open until the update finishes.", flush=True)
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(prog="rvdlab update", description="Upgrade a pip installation of RVDLAB.")
    parser.parse_args(argv)

    try:
        installed = metadata.distribution(PACKAGE)
    except metadata.PackageNotFoundError:
        installed = None
    module_path = Path(__file__).resolve()
    if installed is None or Path(installed.locate_file("rvdlab/update.py")).resolve() != module_path:
        print("rvdlab update requires an installed rvd-railbench release. "
              "Update source checkouts through Git.", file=sys.stderr)
        return 1
    if util.find_spec("pip") is None:
        print("pip is unavailable in this environment. Use the tool that installed RVDLAB to upgrade it.",
              file=sys.stderr)
        return 1

    command = [sys.executable, "-m", "pip", "install", "--upgrade"]
    if site.ENABLE_USER_SITE and module_path.is_relative_to(Path(site.getusersitepackages()).resolve()):
        command.append("--user")
    command.append(PACKAGE)
    print(f"Checking for RVDLAB updates (installed: {installed.version})...", flush=True)
    if sys.platform == "win32" and len(sys.orig_argv) > 1:
        launcher = Path(sys.orig_argv[1])
        if launcher.name.casefold() == "rvdlab.exe":
            return _update_after_windows_launcher(launcher.resolve(), command)

    try:
        return subprocess.run(command, check=False).returncode
    except KeyboardInterrupt:
        return 130
