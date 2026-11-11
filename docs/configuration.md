# Configuration and experiments

## Runtime configuration

`rail_main` uses strict INI files: sections, `key = value`, whole-line `#`/`;`
comments, finite numbers, booleans and comma-separated triples. Unknown or
duplicate keys within one file are rejected. Inline comments, quoting,
includes and environment expansion are unsupported.

```sh
build/release/rail_main --config configs/rail.ini --config configs/pfcr_dqp.ini --config configs/simulation.ini --fast --max-steps 12 --experiment-dir experiments/example
build/release/rail_replay experiments/example
```

Use `.exe` on Windows. Config files apply in order; CLI options override them
regardless of argument order. The standalone executable loads no implicit
config files. The three files in `configs/` list the INI keys, apart from
optional `[experiment] directory`. Every run saves the resolved configuration.

| File | Contents |
|---|---|
| `rail.ini` | Travel, speed, acceleration, timing, tracking, command mode and Hill-to-rail mapping |
| `pfcr_dqp.ini` | Controller, model identity, backend, period and observation warning threshold |
| `simulation.ini` | Plant mode, noise, delay, fault injection, seed, start and step limit |

PFCR runs at a 1 s period. Its trained constants, policy, history and actuator
profile (tau=2 s, scale=.85 per axis, deadzone=.025 N, controller tracking
tolerance=1 m) are fixed. The example rail limits are simulation settings;
configure the runtime and node profiles for the rig being used.

Exit codes are 0 for docking, 1 for a runtime fault or step limit, 2 for
configuration/exception failures, and 3 for rejected initial measurements.
A step limit is recorded as `LIMIT_REACHED` / `MAX_STEPS`.

## RVDLAB profiles

Start `rvdlab` and use Rail Setup to save travel, speed/acceleration limits and
the Hill-to-rail mapping. **Suggest mapping** fits reference approaches to the
rail; **Validate** runs the deployment start set in simulation.

By default, RVDLAB loads the controller and simulation configs plus the active
rail profile, or the example configs if no profile is active. Explicit
`--config FILE` arguments replace that selection. This differs from the
standalone executable's no-implicit-config behavior.

| Data | Location / override |
|---|---|
| Profiles and experiments, installed wheel | `~/.rvdlab/profiles`, `~/.rvdlab/experiments`; base set by `RVDLAB_HOME` |
| Profiles and experiments, source checkout | `<root>/profiles`, `<root>/experiments` |
| Explicit directories | `--profiles`, `--experiments`, then `RVDLAB_PROFILES`, `RVDLAB_EXPERIMENTS` |
| Node connection | `--node`, then `RVDLAB_NODE`, then `<RVDLAB_HOME or ~/.rvdlab>/node/connection.json` |

The archive installers set the profile/experiment directories under
`RVDLAB_HOME`. A node connection folder contains `connection.json` and its CA
certificate. Node JSON configuration is described in
[hardware integration](hardware-integration.md).

## Command modes and frames

`rail.command_mode = arrive` (default) commands a segment ending at the next
controller state, re-anchored to each measurement. `lead` integrates the
reference from realized force, adds `tracking_gain_per_s` times the reference
error to its speed, and places a stop target `lead_s` beyond the period.
The correction uses the remaining speed and acceleration headroom. It uses
encoders when available, otherwise the primary position measurement.

Tracking supervision compares the primary position with the reference.
`sensor_disagreement_m` checks primary position against encoders, and
`MEASUREMENT_REPEATED` detects a bit-identical primary position while encoders
move. On the current hardware bridge, both fields contain encoder positions;
VRPN is a separate node sensor.

Hill coordinates are radial x (docking port on +x), along-track y and
cross-track z. The mapping to rail metres is:

```text
rail[axis[j]] = target[axis[j]] + sign[j] * scale[j] * hill[j]
hill[j] = sign[j] * (rail[axis[j]] - target[axis[j]]) / scale[j]
```

Axes form a permutation, signs are +1/-1 and scales are positive. Defaults are
axes 0,1,2; signs +1,+1,+1; scales 1,.1,.1; target 0,0,0. This experiment scaling
is separate from the node's optional mocap-to-rail affine map.

## Simulation and node runs

| Mode | Behavior |
|---|---|
| `ideal` | Default compatibility model; position jumps to the target |
| `bounded` / `--bounded` | Speed/acceleration-limited motion and braking, integrated in steps of at most 1 ms |
| `plc` / `--plc` | Positioning drive with `plc_delay_s` and `plc_acceleration_m_s2`; stops at its target and accepts retargeting while moving |

`--sim-trace FILE` records plant position, velocity, target and speed every
10 ms in bounded/PLC modes. Encoders are reported unless `encoder = false`.
`repeat_pose_at` freezes the primary pose with fresh timestamps/frame numbers;
`freeze_at` freezes the measurement stream. Read/send failures use zero-based
call counts, including initial read zero; `-1` disables an injection.

`--fast` uses a virtual clock and still runs timestamp checks. Normal execution
uses a monotonic clock and absolute deadlines. Noise, latency and bias are
configurable. A seed reproduces noise within the same standard-library
implementation; recorded-input replay avoids regenerating it.

For hardware, select **rail node** in Autonomous Rendezvous. RVDLAB starts the
bridge, acquires control and uses real time. It overrides `clock_skew_s` to
0.1, `communication_timeout_s` to 0.8 and `measurement_late_s` to 0.5. A run starts
at the measured position; **To start** moves the rail to the configured start
first. **rail node, z simulated** moves X/Y and simulates Z in the bridge.
The node speed floor and feedback path are described in
[hardware integration](hardware-integration.md#feedback-and-bridge-behavior).

## Records and replay

Each run creates a fresh `experiments/<UTC-id>/` directory, or the exact
`--experiment-dir` path. Existing directories are refused.

| File | Contents |
|---|---|
| `metadata.json` | Model/artifact hashes, backend, compiler, build commit/dirty state, seed, clock and rail source |
| `configuration.ini` | Resolved runtime configuration |
| `trajectory.csv` | Original columns followed by reference `rx,ry,rz` and `enc_x,enc_y,enc_z,t_encoder` |
| `initial_state.csv`, `controller.csv` | Full-precision initial state, controller inputs, expected outputs and flags |
| `events.jsonl`, `summary.json` | Transitions, faults, outcome, exit code, steps and interventions |
| `checksums.sha256` | Completion-time integrity manifest for the runtime records above |
| `rail_node.json` | Added by RVDLAB after a node run: simulated axes/start/model and speed floors |

Bridge runs set `metadata.json` to `hardware: true`, `rail: "bridge"`, including
runs against a simulated node or with Z simulated. Check the node profile and
`rail_node.json` for the actual setup. The latter is outside the C++ checksum
manifest; the runtime snapshot does not include the node's hardware profile.
Keep the profile with the experiment's lab records.

`trajectory.csv` is flushed each step. `--log FILE` (default
`pfcr_rail_log.csv`) exports a compatibility copy when the run ends.

`rail_replay EXPERIMENT_DIR [MODEL_DIR]` verifies integrity and replays the
recorded controller inputs, comparing actions/next states at 1e-9 and exact
status/flags. It sends no hardware commands and does not reconstruct physical
motion. Use the same backend/toolchain for exact replay. Initial-read failures
have no controller inputs; incomplete runs without a manifest are rejected.
