from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="rvdlab", description="Terminal interface for RVD-RailBench")
    parser.add_argument("--root", type=Path, help="RVD-RailBench checkout (default: detected)")
    parser.add_argument("--config", type=Path, action="append",
                        help="rail_main configuration file, applied in order; replaces configs/*.ini")
    parser.add_argument("--experiments", type=Path,
                        help="experiment directory (default: $RVDLAB_EXPERIMENTS or <root>/experiments)")
    parser.add_argument("--profiles", type=Path,
                        help="rail profile directory (default: $RVDLAB_PROFILES or <root>/profiles)")
    args = parser.parse_args(argv)
    if (sys.stdout.encoding or "").lower().replace("-", "") != "utf8":
        sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[union-attr]  # block glyphs on non-UTF-8 pipes

    from .app import RvdLabApp

    configs = [path.resolve() for path in args.config] if args.config else None
    RvdLabApp(root=args.root, configs=configs, experiments=args.experiments, profiles=args.profiles).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
