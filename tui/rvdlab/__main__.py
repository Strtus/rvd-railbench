from __future__ import annotations

import argparse
import sys
from pathlib import Path


def check(project) -> int:
    from .project import probe_simulator, system_status

    statuses = system_status(project)
    runs = probe_simulator(project.build)
    print(f"RVD-RailBench {project.version} · {project.root}")
    for status in statuses:
        text = status.text
        if status.label == "Simulator" and status.ok and not runs:
            text = "Built, but does not start on this machine"
        mark = "●" if status.ok and (status.label != "Simulator" or runs) else "○"
        detail = f"  ({status.detail})" if status.detail else ""
        print(f"  {status.label:<22}{mark} {text}{detail}")
    ready = runs and next(s for s in statuses if s.label == "Policy").ok and not project.config_error
    return 0 if ready else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="rvdlab", description="Terminal interface for RVD-RailBench")
    parser.add_argument("--root", type=Path, help="RVD-RailBench installation or checkout (default: detected)")
    parser.add_argument("--config", type=Path, action="append",
                        help="rail_main configuration file, applied in order; replaces configs/*.ini")
    parser.add_argument("--experiments", type=Path,
                        help="experiment directory (default: $RVDLAB_EXPERIMENTS, else ~/.rvdlab or <root>/experiments)")
    parser.add_argument("--profiles", type=Path,
                        help="rail profile directory (default: $RVDLAB_PROFILES, else ~/.rvdlab or <root>/profiles)")
    parser.add_argument("--node", type=Path,
                        help="connection.json of a rail node (default: $RVDLAB_NODE, else ~/.rvdlab/node/)")
    parser.add_argument("--check", action="store_true",
                        help="print the system status and exit; exit code 1 if the simulator or policy is not ready")
    parser.add_argument("--version", action="store_true", help="print versions and exit")
    welcome = parser.add_mutually_exclusive_group()
    welcome.add_argument("--welcome", dest="welcome", action="store_true", default=None,
                         help="replay the first-launch welcome animation")
    welcome.add_argument("--no-welcome", dest="welcome", action="store_false",
                         help="skip the welcome animation for this launch")
    args = parser.parse_args(argv)
    if (sys.stdout.encoding or "").lower().replace("-", "") != "utf8":
        sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[union-attr]  # block glyphs on non-UTF-8 pipes

    configs = [path.resolve() for path in args.config] if args.config else None
    if args.check or args.version:
        from . import __version__
        from .project import ProjectError, load_project

        try:
            project = load_project(args.root, configs, args.experiments, args.profiles)
        except ProjectError as error:
            print(error, file=sys.stderr)
            return 2
        if args.version:
            print(f"rvdlab {__version__} · runtime {project.version} · {project.root}")
            return 0
        return check(project)

    from .app import RvdLabApp

    RvdLabApp(root=args.root, configs=configs, experiments=args.experiments, profiles=args.profiles,
              node=args.node, welcome=args.welcome).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
