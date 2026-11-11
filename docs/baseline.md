# Numerical baseline

This records the original controller at `4670f55`, measured on 2026-11-06.
The I/O and CLI behavior below describes that version. Current operation is
documented in [configuration](configuration.md) and
[hardware integration](hardware-integration.md).

## Original measurements

Original commit: `4670f55` (Add rail deployment controller).

Windows / MinGW GCC 13.2, Release, Ninja: configured and built successfully;
the original `golden_builtin` CTest passed.
Commands: `cmake -S . -B build-baseline -G Ninja`,
`cmake --build build-baseline`, `ctest --test-dir build-baseline --output-on-failure`.

The policy test covers 1,000 observations (maximum action difference 2.98e-7,
limit 1e-5). All eight forced-action controller runs pass: observation limit
1e-6, force/state limit 1e-9, identical termination and safety flags.
Free-running results are diagnostic: runs 1 and 3 differ at decision boundaries.
Their original reference data and tolerances are retained.

Numerical core: CWH, history, CBF-QP, backup and actuator emulation are in
`src/pfcr_controller.cpp`; policy inference is in two separate backends.
The original app owns mapping, configuration, wall-clock scheduling, CSV,
timestamp checks and RailIO. Simulated I/O teleports to its target; LabRailIO
contains TODO methods and cannot operate hardware. The loop checks freshness,
duplicate frames, tracking, target travel and speed, but not finite values,
acceleration or out-of-order frames. Missing acquisition stamps fall back to
receipt time. `--fast` bypasses wall-clock timestamp checks.

Original floating-point flags: GCC `-ffp-contract=off`, MSVC `/fp:precise`.
No fast-math. Original CLI returns 1 on `MAX_STEPS`, including intentional short
runs. A 12-step zero-noise simulation trace was retained outside the repository
for pre/post-refactor comparison. Windows timer resolution uses winmm; the
controller itself is platform neutral. ONNX Runtime is optional, located via
`ONNXRUNTIME_ROOT`; normal tests require no downloads.

The policy binary, ONNX model, generated parameter header and Golden CSV files
retain their original contents.

Linux baseline was subsequently reproduced from a separate detached checkout of
`4670f55` (Ubuntu 24.04 / GCC 13.3, Release, Ninja): the original build and
`golden_builtin` also pass. The original checkout was not modified.

## Current regression contract

The original mathematics, generated constants and Golden data remain covered
by `tests/regression/protected_inputs.json`. The 12-step application trace is
stored in `tests/regression/application_baseline.json`; its comparison allows
one last printed CSV decimal (1e-6). Golden tolerances remain as listed above.

Subsequent changes add bounded/PLC simulation, lead commands, RVDLAB and the
rail-node hardware path. Their tests supplement this numerical baseline;
they do not replace its reference data. See [development](development.md) for
the current test commands.
