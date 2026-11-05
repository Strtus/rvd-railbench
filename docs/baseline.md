# Baseline (2026-11-06)

Original commit: `4670f55` (Add rail deployment controller).

Windows / MinGW GCC 13.2, Release, Ninja: configured and built successfully;
the original `golden_builtin` CTest passed. No failing gating tests.
Commands: `cmake -S . -B build-baseline -G Ninja`,
`cmake --build build-baseline`, `ctest --test-dir build-baseline --output-on-failure`.

The policy test covers 1,000 observations (maximum action difference 2.98e-7,
limit 1e-5). All eight forced-action controller runs pass: observation limit
1e-6, force/state limit 1e-9, identical termination and safety flags.
Free-running results are diagnostic, as in the original test: runs 1 and 3
already differ at decision boundaries. This refactor must not hide that drift,
relax tolerances, or regenerate Golden data.

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

Policy binary, ONNX, generated parameter header and all Golden CSV files are
protected inputs. Preserve their contents, even if their organization changes.
