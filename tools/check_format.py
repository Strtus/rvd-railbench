"""Check new/refactored code without rewriting the protected numerical core."""
import argparse
from pathlib import Path
import subprocess
parser = argparse.ArgumentParser()
parser.add_argument("--executable", default="clang-format-18")
parser.add_argument("--fix", action="store_true")
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
paths = []
for directory in ("apps", "include/rvd", "src/core", "src/control", "src/runtime", "src/simulation", "tests/unit", "tests/faults"):
    paths.extend(p for p in (root / directory).rglob("*") if p.suffix in (".cpp", ".hpp"))
paths.extend(root / name for name in ("src/policy_builtin.cpp", "src/policy_onnx.cpp", "tests/test_support.hpp"))
subprocess.run([args.executable, *(["-i"] if args.fix else ["--dry-run", "--Werror"]), *map(str, sorted(paths))], check=True)
