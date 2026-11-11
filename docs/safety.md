# Safety and stop behavior

PFCR's CBF-QP, backup recovery, CWH dynamics, actuator model and trained weights
are unchanged. Runtime supervision checks controller execution; rail-node
supervision checks hardware operation.

## Controller runtime

`SafetySupervisor` checks finite measurements, travel, commanded speed and
per-period acceleration, tracking error, timestamps, I/O, command timeout,
controller outputs and execution deadlines. It latches the first fault and
prevents further commands. The C++ runtime states are:

| From | Allowed next states |
|---|---|
| INITIALIZING | READY, FAULT |
| READY | RUNNING, STOPPED, FAULT |
| RUNNING | HOLD, STOPPED, FAULT |
| HOLD | READY, STOPPED, FAULT |
| FAULT | STOPPED |
| STOPPED | none |

`rail_main` exits on fault, docking or the step limit. It does not reset a fault
or resume from HOLD. The step limit is recorded as `LIMIT_REACHED` with exit
code 1. An exit guard requests hold; on the bridge this releases the node lease.

`MEASUREMENT_REPEATED` detects a repeated position while encoders move;
`SENSOR_DISAGREEMENT` checks the position/encoder difference. The current node
bridge supplies encoders to both fields, so these are not independent mocap
checks on hardware runs. The C++ `HardwareSafetyInput` interface is also not
connected to this bridge; PLC checks are performed in rail-node.

## Node supervision

One control thread owns device reads, commands, arming and stopping. Node events
are written on a separate thread; Windows QuickEdit is disabled when serving.
TLS connections are handled by request threads.

Stream commands must fit the axis travel and speed limits and lie within
`speed * horizon_s + tolerance` of the measured position. Waypoint tasks send
intermediate targets `task_lead_s` ahead, clipped to axis travel limits.
The target horizon bounds the requested travel, rather than measuring braking
distance. The bridge's speed floor is applied after C++ command checks; use
the node profile and drive settings to set the hardware operating limits.

| Setting | Default |
|---|---|
| `poll_s` | 0.2 s; profile may override |
| `lease_ttl_s` | 3 s |
| `horizon_s` | 3 s |
| `task_lead_s` / `task_refresh_s` | 2 s / 1 s |
| `max_sample_age_s` | 1 s |
| `stall_timeout_s` | 3 s; profile may override |
| `drive_dropout_s` | 1 s |
| `task_timeout_s` | 900 s |
| `arrival_samples` | 3 |

Override these in `node.json`'s `settings`. Lease expiry, stream deadline expiry
and operator stop revoke control. Feedback loss, motion stalls, sustained drive
disable reports and PLC interlock errors latch faults. A lease still unarmed
after `wait_s + 5 s` is revoked. Stop delivery is retried after communication
failures, and the driver checks standstill.

Node faults can be reset after addressing their cause. Init enables remote
reset; with `remote_reset: false`, reset requires an authenticated loopback connection.
Reset does not restart a task or controller run.

## Held axes and parking

Servo axes marked `hold: true` stay enabled through stops, lease releases,
faults and resets. This keeps a gravity-loaded platform supported. A drive that
has already dropped out is released during reset. On startup, the node takes
over held axes whose drives are still enabled.

Set `park` to a position where the axis can rest unpowered and `park_speed`
to its parking speed (default: 20% of the axis speed limit). Parking releases
the drive only after arrival. Normal shutdown attempts to park enabled held
axes; a second Ctrl+C cancels parking and keeps them holding. Failed parking,
or a held axis without a park position, also leaves the drive enabled.

The driver latches zero speed before enabling a disabled servo and waits for
three consecutive enabled reports, with a default 5 s enable timeout. Brief
missing reports are tolerated up to `drive_dropout_s`.

## Experiment limits

Configure register maps, units, travel, speeds, park positions and interlocks
for the actual rig. Physical E-stop, drive protections and PLC watchdogs remain
independent of the host process. Software holding requires drive power.

Simulation and Golden tests validate software behavior; real runs require
measured settings and recorded hardware results. Label XY-real/Z-simulated
runs accordingly. The original free-running Golden drift in runs 1 and 3 is
retained as documented in [baseline](baseline.md).
