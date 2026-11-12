# Development

RVD-RailBench contains the C++ controller/runtime, the RVDLAB terminal interface
(`tui/`), and the rail-node hardware service (`node/`). See
[configuration](configuration.md) for runs and records,
[hardware integration](hardware-integration.md) for node setup, and
[safety](safety.md) for stop and fault behavior. Original numerical results
are retained in [baseline](baseline.md).

## Install and run

Python 3.10+ is required for RVDLAB and rail-node. Install the combined package
in a Python environment, or use pipx/uv:

```sh
python -m pip install "rvd-railbench[node]"
rvdlab --check
rvdlab
```

The package includes `rail_main`, `rail_replay`, the policy, default configs,
`rvdlab`, and `rail-node`. The `node` extra adds `cryptography`, used to create
the node's TLS identity. `rvdlab --check` checks the installation and runs a
short simulation; it does not probe hardware. `rvdlab --version` reports the
interface and runtime versions.

The first interactive launch opens with only the Braille orbital animation.
The outline appears over 3 seconds, followed by 2.5 seconds of full-size orbital
motion. The comet uses Kepler's equation to move faster near the focus and slower
far away on each stylized ellipse. The logo then shrinks over 1.6 seconds, and a
blinking cursor types the RVDLAB wordmark to its right. The compact logo and
wordmark are both six rows high, and the comet keeps orbiting. Braille cells
shared by multiple orbits or the central hub stay dim so highlighting does not
spill onto adjacent curves. There are no captions or key prompts.
Press any ordinary key at any time to enter the normal rail setup or home screen.
RVDLAB then records `~/.rvdlab/welcome-seen` (under `RVDLAB_HOME` when set), so later
launches go straight to the interface. `rvdlab --welcome` replays the animation;
`rvdlab --no-welcome` skips it for one launch. `--check` and `--version` never
play it. Ctrl+Q still quits the application.

The full-size animation fits in a 108-column, 27-row terminal; it scales down in
smaller windows. The final six-row lockup fits in 77 columns; narrower terminals
use smaller matching-height artwork. Use a font with Unicode Braille support, such as
Cascadia Mono. The animation is drawn by the Python interface on every platform
and does not require PowerShell.

While a model run controls the rail node, a thick border surrounds the terminal:
steady green for normal control, steady yellow while the latest telemetry step
reports CBF intervention, and flashing red for a runtime or bridge fault. Red
alternates bright and dark every half second and stays visible after a failed
run until the next run starts. CBF release returns the border to green; normal
docking, the step limit and operator stop clear it after bridge cleanup. The
border follows page and dialog changes. Both rail-node modes (including simulated
Z) use this cue; local simulations, offline comparisons and manual moves do not
trigger it.

`install.sh` and `install.ps1` offer release-archive installation with SHA-256
checks and a source-build fallback. They currently install only `tui/` into
their environment; node access also needs `node/` installed there. The combined
Python package includes both.

## Build from source

Requires C++17 and CMake 3.16+; presets require CMake 3.21+ and Ninja. On Windows,
use an x64 Visual Studio developer shell or MinGW with its runtime DLLs on PATH.
The standalone C++ simulator and replay tool do not require Python at runtime.

```sh
cmake --preset release
cmake --build --preset release
ctest --preset release
python -m pip install -e "./node[test]" -e "./tui[test]"
rvdlab --root . --check
```

`dev` builds Debug; `release` builds Release; `minimal` omits tests and the
Windows timer-resolution request. Without presets, use `cmake -S . -B build-local`,
then build and run CTest in that directory.

`RVD_SIMULATION_ONLY=OFF` and the legacy `--lab` route remain unsupported.
Despite its name, `RVD_SIMULATION_ONLY=ON` does **not** disable `--bridge`:
hardware runs use the Python node through that bridge. `RVD_WINDOWS_TIMER`
controls winmm use.

### Preview control status in the TUI

After building and installing the source checkout, run:

```sh
python tools/preview_control_states.py
```

The terminal opens directly on a local simulation and starts after two seconds.
It runs PFCR from Hill `(18, 0.3, -0.2)` m with seed 12345 and a one-second clock:
normal control is green, CBF first intervenes at step 11 and turns the border
yellow, and an injected send failure at step 30 leaves a flashing red border.
F5 replays, F6 stops, and Escape exits. The launcher enables the border for this
demo only, ignores saved node connections, and records `hardware: false`.
Records go to `out/control-status-demo/`. Add `--check` to verify the same
scenario headlessly at full speed.

## Architecture

| Component | Responsibility |
|---|---|
| `pfcr_core` / `rvd::pfcr` | PFCR mathematics and policy inference |
| `rvd_runtime` / `rvd::runtime` | Configuration, frame mapping, supervision, simulation, bridge and records |
| `rail_main`, `rail_replay` | Run a controller; verify recorded controller inputs |
| `tui/rvdlab` | Profiles, motion control, rendezvous, telemetry and experiment management |
| `node/rvd_railnode` | Exclusive hardware ownership, Modbus, VRPN, HTTPS and motion supervision |

Controllers implement `IController`. The PFCR adapter accepts Hill state;
`pfcr::Controller` constructs the 36-element policy observation and owns the
model-derived velocity, history, safety logic and actuator state.

## Tests and checks

Run the Python suites separately: both use a top-level `conftest` module.

```sh
python -m pytest -c node/pyproject.toml node/tests -q
python -m pytest -c tui/pyproject.toml tui/tests -q
python tools/check_format.py --executable clang-format-18
```

CTest covers Golden regression, configuration, runtime faults, simulation,
bridge integration and replay. Node tests use simulated axes, fake registers,
a Modbus PLC emulator and a local VRPN server. Interface tests exercise Textual
screens and node runs; build the C++ executables first to include those runs.
These suites use local test services, not lab equipment.

CI builds GCC/Linux and MSVC/Windows in Debug and Release, runs node tests on
Python 3.10/3.13 on both systems, and runs the interface on Linux/Python 3.12.
It also checks formatting, clang-tidy and the optional ONNX backend. For local
clang-tidy, configure `dev` with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`, then run
`run-clang-tidy-18 -p build/dev -quiet 'src/(core|control|runtime|simulation)/'`.

`protected_inputs` checks the original mathematics, constants and Golden data.
Golden limits remain 1e-5 for policy, 1e-6 for forced-action observations and
1e-9 for force/state, with exact status and safety flags. The 12-step application
fixture allows one last printed CSV decimal (1e-6). Preserve the existing
floating-point order (`-fno-fast-math -ffp-contract=off` or `/fp:precise`).

## Optional ONNX

Install ONNX Runtime separately; CI uses 1.20.1.

```sh
cmake -S . -B build-onnx -DRVD_ENABLE_ONNX=ON -DONNXRUNTIME_ROOT=/path/to/onnxruntime
cmake --build build-onnx --config Release
ctest --test-dir build-onnx -C Release --output-on-failure
```

Presets disable ONNX unless overridden. A fresh non-preset configuration also
enables it when `ONNXRUNTIME_ROOT` is supplied. Requested ONNX support fails
explicitly if unavailable. Windows DLLs are staged beside the executables.

## Packaging and provenance

The root `pyproject.toml` builds the combined package with scikit-build-core.
Version comes from `CMakeLists.txt`; keep the Python package versions in sync.
The release workflow builds Linux/Windows archives, platform wheels on
Linux/Windows/macOS, and an sdist. Tags publish GitHub releases; PyPI publishing
also requires the repository's `PYPI_PUBLISH` variable and trusted publisher.
Intermediate release artifacts are retained for seven days.

The PyPI description comes from the root `README.md`; the summary, project links,
keywords, and classifiers live in `pyproject.toml`. Keep README image and document
links absolute so they also work on PyPI. The release workflow runs
`twine check --strict` on wheels and the sdist before uploading build artifacts.
PyPI stores metadata with each release, so updating the public description
requires a new version and release tag.

Reconfigure CMake before producing a research run: metadata captures commit and
dirty state at configuration time. A source installation is needed for changes
on `main` that have not yet been included in a release tag.
