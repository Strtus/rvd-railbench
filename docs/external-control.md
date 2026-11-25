# External control with Simulink

The external control mode lets Simulink compute the next position and velocity
reference. The RVDLAB runtime owns the experiment clock, maps between Hill and
rail coordinates, supervises the move, and records the result. It can use the
local rail simulation or the existing rail-node connection.

## Run the simulated example

Build the runtime with the [source presets](development.md#build-from-source).
Use a Python interpreter with the RVDLAB dependencies installed. The example was tested with MATLAB
R2026a and Simulink. In MATLAB, change the current folder to the example directory, examples/simulink. Then run the following command.

```matlab
[out, experimentDir] = run_rvdlab_simulink_demo;
```

The example runs a small CWH rendezvous to the Hill origin. The example rail
profile maps the origin to X = 50 mm and Y = 15 mm. The model uses live position
feedback from the simulated rail and its own integrated velocity. It runs for
17 one-second steps and plots measured position and the position reference.
The same MATLAB System block, `RvdLabExternalIO`, can be used with a different
Simulink model. It outputs a 3-element Hill position in metres and takes a
6-element reference vector containing position in metres followed by signed
velocity in metres per second.

The script uses the current checkout's Python modules and compiled runtime.
If MATLAB needs a different Python interpreter, set the `RVDLAB_DEMO_PYTHON`
environment variable to that interpreter's full path before running the example.

## Start an external session

A custom launcher can start a local experiment with the following command.

```text
rvdlab external --root . --config examples/simulink/rail-demo.ini --max-steps 17 --name my-simulink-model --ready-file session.json
```

Pass a fresh ready file path in a private directory. Its JSON object contains
`host`, `port`, `period_s`, `experiment_dir`, and a one-time `token`. MATLAB and the
RVDLAB launcher run on the same computer. The listener binds to 127.0.0.1 and
accepts a single authenticated client for the session. Connect and send
`HELLO <token>` before the 60-second startup window closes. The launcher starts `rail_main` after
authentication and removes the ready file as the session ends.

Without `--node` the runtime uses its rail simulator. For the hardware path, pass
`--node` with the node's `connection.json` and select the validated runtime profile
using `--config` or an active rail profile. The profile must explicitly supply the
rail travel, speed and acceleration limits and the Hill mapping. The optional
`--experiments` flag sets the directory where runs are saved. The `--name` value is
stored in the record so different Simulink models can be identified later.

For hardware commissioning, see [hardware integration](hardware-integration.md)
and [stop behavior](safety.md). The node acquires exclusive X/Y/Z control after
the external client authenticates. The runtime checks travel, commanded speed and
acceleration, tracking error, sample timestamps, and deadlines before continuing
the run. The hardware path uses the speeds checked by the runtime for moving axes.
The node can reject a request below the drive's minimum speed. An axis can use
zero reference speed to hold an already reached encoder target. The rail node must support the external mode's separate
setpoint and watchdog deadlines. Upgrade the node before enabling this mode on
an installation that uses an older rail-node release.

## Step exchange

Send and receive LF-terminated ASCII on the same TCP connection. After the client sends HELLO, the
runtime sends the first STATE line and paces the experiment in wall-clock time. Set the
Simulink communication block to the period in the ready file (one second for
this implementation). Solver steps inside the rest of the model can be faster.

```text
STATE k t_acquired_s frame_id hx hy hz
REF k hx hy hz hvx hvy hvz
```

STATE contains a zero-based step, the sample time in the runtime's monotonic
clock, the feedback frame number, and the measured Hill position. Reply with
REF for the same step. Its position is the target for the next tick and its
signed velocity specifies the axis speeds toward that target after mapping to
rail coordinates. The controller should produce a feasible target and speed together. A missed
deadline, malformed reference, lost connection, or supervisor fault ends the
experiment and requests a stop.

For a custom model, use the ready file to set the MATLAB System block's `Port`,
`Period`, and `Token` properties. Convert the token to `uint8` as in the example
launcher. For a fixed-length run, set `ExpectedSteps` to the runtime's
`--max-steps` and set the stop time to `(ExpectedSteps - 1) * Period`.
The runtime checks the final accepted reference on the next rail sample before
marking the experiment complete. This final sample is in the trajectory record.

The client may send `STOP k` in place of a REF to stop normally. On termination,
the runtime attempts to send `END OUTCOME REASON`. A normal configured step limit
reports `END COMPLETE MAX_STEPS`. A client stop reports `END STOPPED CLIENT_STOP`.
A fault reports `END FAULT` followed by its reason. The summary on disk retains
the outcome if the client loses its connection.

## Inspect a run

The [Experiment Manager](../README.md#experiment-records-and-replay) lists external sessions
with built-in controller runs. `trajectory.csv` records measured positions,
references, actual command positions and speeds in rail units, tracking error,
timing, and status. `controller.csv` carries the Hill-coordinate measurements and
external references. `metadata.json`, `configuration.ini`, `events.jsonl`,
`summary.json`, and `checksums.sha256` capture context and run integrity. The logs support
trajectory and tracking-error comparison across runs. PFCR replay is limited
to its built-in controller.
