# Configuration and experiments

Configuration uses a small strict INI schema to avoid a runtime dependency or a
home-grown YAML parser. YAML is not supported. Files contain `[section]`,
`key = value`, whole-line `#`/`;` comments, finite decimal numbers, `true`/`false`,
and comma-separated triples. Quoting, inline comments, includes and environment
expansion are unsupported. Unknown keys, duplicate keys within one file,
malformed values and incompatible profiles fail before I/O.

```sh
build/release/rail_main --config configs/rail.ini --config configs/pfcr_dqp.ini --config configs/simulation.ini --fast --max-steps 12 --experiment-dir experiments/example
build/release/rail_replay experiments/example
```

Use `.exe` on Windows. A limited run returns **1** with `MAX_STEPS`, preserving
the original CLI. Docking returns 0, runtime faults 1, configuration/exception
failures 2, initial measurement failures 3. Config files apply in order; CLI
options override them regardless of argument order. No implicit file is loaded:
absent files use validated compatibility defaults. The three checked-in files
are the complete configurable schema except optional `[experiment] directory`.
Every run stores a fully expanded, reloadable snapshot.

`rail.ini` defines travel [m], speed [m/s], acceleration [m/s^2], freshness,
clock-skew/jitter/communication thresholds [s], tracking tolerance [m] and
latency compensation. These are simulation defaults, not measured hardware
ratings. `pfcr_dqp.ini` selects controller, artifact identity, backend, period
and observation warning threshold. PFCR period must be exactly 1 s. The compiled
CWH/CBF/backup constants, 36-to-3 policy, history and original app actuator
profile (tau=2 s, scale=.85 per axis, deadzone=.025 N, controller tracking
tolerance=1 m) cannot be changed through these files.

## Frames

Hill coordinates are radial x (docking port on +x), along-track y, cross-track z.
Rail coordinates are physical Cartesian axes in metres. The mapping is exactly:

```
rail[axis[j]] = target[axis[j]] + sign[j] * scale[j] * hill[j]
hill[j] = sign[j] * (rail[axis[j]] - target[axis[j]]) / scale[j]
```

Axes must be a permutation, signs +1/-1, scales finite/positive and invertible.
Defaults remain axes 0,1,2; signs +1,+1,+1; scales 1,.1,.1; target 0,0,0.
This anisotropic experiment mapping is not a rigid-body sensor extrinsic.
Motion-capture, camera, LiDAR 1/2 and target-body frames have separate enum tags.
No calibration between those frames is presumed or used: measured extrinsics,
pose conventions and sensor time calibration are required before integration.
`Pose` stores metres and a w,x,y,z quaternion. PFCR rejects non-Hill estimates.

## Simulation

`--bounded` (or `simulation.mode = bounded`) activates finite-duration dynamics:
position and velocity evolve with bounded speed and acceleration in <=1 ms
integration steps, including bounded braking on hold. Arrival is not guaranteed
for infeasible commands; tracking supervision detects missed targets. The
original teleport model remains the default `ideal` compatibility mode.

`--fast` uses a deterministic virtual monotonic clock; it now exercises freshness
and frozen-frame checks too. Ordinary execution uses `steady_clock` and sleeps
to absolute deadlines. Measurement noise, latency, constant sensor bias,
read/send failure counts, and freezing are configurable. Bounded mode samples
position history at acquisition time; ideal mode preserves the original
teleport/read semantics with latency stamps. Acquisition before startup uses
the initial position. `freeze_at` preserves the original frame-count convention;
read/send failures use zero-based call counts (initial read is call zero).
`-1` disables injection. Seeds reproduce noise with the same standard-library
implementation; `std::normal_distribution` is not bit-identical across all
standard libraries. Recorded-input replay avoids regenerating noise.
This is a bounded kinematic test model, not a validated drive/friction/contact
model or evidence of sim-to-real fidelity.

## Records and replay

Every run creates a new `experiments/<UTC-id>/` directory, or the exact fresh
`--experiment-dir` path. Existing directories are refused. Files are:

- `metadata.json`: controller/model identity, artifact SHA-256, backend, compiler,
  build commit/dirty state, UTC creation time, seed and clock mode.
- `configuration.ini`: resolved runtime configuration, including simulation seed.
- `trajectory.csv`: original CSV column order and six-decimal presentation.
- `initial_state.csv` and `controller.csv`: full-precision initial state and
  actual controller inputs, expected outputs, termination and safety flags.
- `events.jsonl`: lifecycle transitions, safety interventions, faults and outcome.
- `summary.json`: final outcome/reason, exit code, steps and interventions.
- `checksums.sha256`: completion-time integrity of configuration, inputs and logs.

`--log FILE` and the original `pfcr_rail_log.csv` default remain compatibility
exports at the end of a run. For live consumers, use the experiment's
`trajectory.csv`, which is flushed each step. `--log` may name that exact file;
it cannot overwrite other structured files in the experiment directory.

`rail_replay EXPERIMENT_DIR [MODEL_DIR]` verifies integrity, restores initial
state, replays recorded inputs and checks actions/next states at 1e-9 with exact
status/flags. It creates no RailIO and sends no commands. Exact replay is
expected for the same backend/toolchain; cross-platform policy rounding may
require investigation rather than a weakened tolerance. It does not replay
physical dynamics, validate unrecorded sensor processing, or prove hardware
safety. Initial-read failures have no controller inputs and cannot be replayed.
Interrupted runs without a completion manifest are rejected. SHA-256 detects
accidental changes; it is not a signature or an adversarial trust mechanism.
