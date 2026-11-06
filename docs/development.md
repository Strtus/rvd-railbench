# Development

Requires a C++17 compiler and CMake 3.16+ (3.21+ for presets); Ninja for presets.
Tests additionally use Python 3.9+ standard library. Runtime has no Python,
network or third-party dependency unless ONNX is enabled. On Windows use an
x64 Visual Studio developer shell, or a MinGW installation with runtime DLLs
on PATH. CI covers GCC/Linux and MSVC/Windows in Debug and Release.

```sh
cmake --preset release
cmake --build --preset release
ctest --preset release
# or the original configuration style:
cmake -S . -B build-local -DBUILD_TESTING=ON
cmake --build build-local --config Release
ctest --test-dir build-local -C Release --output-on-failure
```

`dev` is Debug, `release` is Release, `minimal` disables tests and Windows timer
resolution. `RVD_SIMULATION_ONLY=OFF` is a configuration error: no hardware
backend exists. `RVD_WINDOWS_TIMER` controls winmm use. Tests always operate on
simulation or files; they never discover devices. Generated runs are ignored
under `experiments/` or `build*/`.

The `pfcr_core` / `rvd::pfcr` target retains the algorithm and public PFCR API.
`rvd_runtime` / `rvd::runtime` supplies adapters, supervisor, configuration,
simulation and experiments. `rail_main` and `rail_replay` are small entry points.
New controllers implement `IController`: initialize an artifact, reset state,
update an `Observation`, report status/diagnostics. An observation optionally
contains a state estimate and/or typed camera, point-cloud and pose samples;
there is no universal 36-element observation requirement. Only the PFCR adapter
requires Hill state and uses model-derived velocity, exactly as the original app.
Its history, safety logic and actuator state remain owned by `pfcr::Controller`.

## Numerical regression

The original mathematical source and generated parameter header are retained
at their existing paths and checked by `protected_inputs`. Golden CSV files
remain under `golden/`; none were regenerated. `golden_builtin` retains its
original thresholds: policy 1e-5; forced-action observation 1e-6; force and state
1e-9; exact termination and safety flags. Free-running drift is reported, not
judged, as in the original test (see `baseline.md`). A separate application
fixture was captured from commit `4670f55` before refactoring, with zero noise
and 12 steps; comparison permits one last printed CSV decimal (1e-6).

Target-level `-fno-fast-math -ffp-contract=off` or `/fp:precise` protects arithmetic
order. Do not enable fast-math or change trained constants to repair a failing
test. Legacy controller mathematics is excluded from formatting churn. Native
artifact serialization assumes the original little-endian IEEE float format;
current validation is on x64 platforms.

CTest labels: `unit`, `integration`, `regression`, `faults`, and optional `onnx`.
Unit tests use throwing checks, not assertions disabled by Release builds.
Fault tests cover clocks, frozen/out-of-order sensors, physical operating
limits, missing status, invalid outputs, timeouts, simulator dynamics and I/O
failures. Integration tests check command-line failure paths, baseline
compatibility, repeated deterministic logs, metadata, and replay corruption.

## Optional ONNX

Install a compatible ONNX Runtime package separately (CI pins and verifies
1.20.1). Normal configure/build/test steps never download it.

```sh
cmake -S . -B build-onnx -DRVD_ENABLE_ONNX=ON -DONNXRUNTIME_ROOT=/path/to/onnxruntime
cmake --build build-onnx --config Release
ctest --test-dir build-onnx -C Release --output-on-failure
```

On a fresh non-preset configuration, setting `ONNXRUNTIME_ROOT` also enables the
backend for legacy workflows. Presets explicitly disable it; override
`RVD_ENABLE_ONNX=ON` there. Requested ONNX without support fails instead of
silently switching backend. Signature, types, dimensions, names and SHA-256
are verified. Windows DLLs are staged beside executables. Policy files moved
to `policies/<model-id>/`; their bytes are unchanged. `--model-dir` remains
supported; the original default `model/<model-id>` redirects if absent.

## Quality checks

```sh
python3 tools/check_format.py --executable clang-format-18
cmake --preset dev -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
run-clang-tidy-18 -p build/dev -quiet 'src/(core|control|runtime|simulation)/'
```

Use `--fix` to apply the project's style to new/refactored files. CI runs both
checks. The original controller source and Golden test are deliberately kept
out of style-only rewrites. Reconfigure CMake after committing before producing
a research run: metadata records the commit/dirty state at configuration time,
not a guess about a later checkout.
