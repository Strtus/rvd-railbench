# Hardware integration

The hardware path is:

```text
rail_main <-> RVDLAB bridge <-> HTTPS rail-node <-> Modbus TCP PLC
                                      ^
                                      |
                                  VRPN mocap
```

`rail-node` runs on the computer wired to the PLC and owns device I/O. Other
computers connect with a personal token and the node's CA certificate. RVDLAB
launches `rail_main --bridge PORT` against a local TCP peer and forwards its
commands to the node. The legacy C++ `--lab` route is unused.

## Set up a node

Install `rvd-railbench[node]`, then create a node directory outside the checkout:

```sh
rail-node init --directory rail-node-demo --address localhost --users alice
rail-node serve --config rail-node-demo/node.json
```

Without `--profile`, init creates a simulated rail. For a PLC, supply
`--profile /path/to/profile.json` and the hostname/IP that clients will use.
Init creates `node.json`, the profile, TLS keys/certificates, `users.json`, and
an `access/<name>/` folder for each user, including `local` for the node PC.
Give each user only their access folder. Init refuses an existing directory.

`serve` starts read-only unless `--allow-motion` or `node.json`'s `allow_motion`
is true. The generated config sets it false. Before starting the service, use
`rail-node probe --profile /path/to/profile.json` to read the device without
writing. Once the node owns I/O, use its status command instead.

```sh
rail-node status --config rail-node-demo/node.json --diagnostics
rail-node add-user bob --config rail-node-demo/node.json
rail-node remove-user bob --config rail-node-demo/node.json
```

User changes take effect without restarting. Remote clients use
`--connection /path/to/access/alice/connection.json` instead of `--config`.
A non-loopback listener requires TLS.

## Hardware profile

Profiles are JSON; `node/rvd_railnode/profile.py` validates them. Keep site
addresses, register maps and access credentials outside the repository.

| Section | Fields |
|---|---|
| `driver` | `modbus` or `simulator` |
| `modbus` | `host`, `d_base`, `m_base`; optional `port`, `unit`, `d_stride`, `word_order`, `timeout_s` |
| `axes` | A subset of `x`, `y`, `z`, `pitch`, `yaw`, `roll`; linear units are m, angular units rad |
| Each axis | `max_speed`, `tolerance`, optional `motion`, `hold`, `park`, `park_speed` |
| Modbus axis | `type` (`pulse`/`servo`), `target_d`, `speed_d`, `encoder_d`, `zero`, `scale`, `min_count`, `max_count` |
| Pulse control | `start_m`, `stop_m` |
| Servo control | `enable_m`, `stop_m`, `enabled_d`, `closed_d` |
| Optional axis checks | `ready` entries with `d`/`value`, `quickstop_m`, `pending_d`, `sample_count_d` |
| `interlock` | Optional `identity` (`d`/`value`) and `emergency_m` |
| `diagnostics` | Labels mapped to D register numbers; shown by `status --diagnostics` |
| Simulator axis | `minimum`, `maximum`; optional `start`, `acceleration`, `resolution` |

For Modbus axes, position is `(count - zero) * scale`; the sign of `scale`
sets direction. Count limits determine travel limits. The driver reads signed
32-bit D values with configurable word order and maps M bits to coils.
Diagnostics are read after positions, using block reads with single-read
fallback; their errors are reported separately from position feedback.

Profile `poll_s` and `stall_timeout_s` feed node settings; `enable_timeout_s`
and `stop_timeout_s` configure the driver. Other supervision settings go in
`node.json`'s `settings` object. Defaults and stop behavior are in
[safety](safety.md).

## Control and operation

Protocol `rail-node/1` grants one exclusive lease at a time on selected axes.
It arms those axes before returning the lease. A client either streams position
and speed setpoints with increasing sequence numbers and deadlines, or submits
waypoint tasks. The Python client maintains a heartbeat. Any authenticated user
can stop the node.

On a motion-enabled node:

```sh
rail-node move --connection /path/to/connection.json --by x=0.01 --speed 0.01
rail-node stop --connection /path/to/connection.json
rail-node park --connection /path/to/connection.json
rail-node reset --connection /path/to/connection.json
rvdlab --node /path/to/connection.json
```

`move` also accepts absolute `--to AXIS=VALUE` targets. It releases its lease
on completion; held axes remain enabled. `park` moves enabled held axes to
configured resting positions, then releases their drives. `reset` clears a
node fault; remote reset is enabled by init and can be disabled with
`remote_reset: false`.

RVDLAB Motion Control offers moves, Level platform, Park platform and Reset
fault. Autonomous Rendezvous offers simulation, rail node, and rail node with
Z simulated. A node run starts where the rail stands; use **To start** to move
first. F9 stops the rail on either page; F6 also stops a rendezvous run.
Level platform moves held axes to zero at their parking speeds and keeps them
enabled; Park platform moves them to their configured rest positions and releases them.

## Feedback and bridge behavior

`GET /v1/state?fresh=1` waits for a poll started after the request and returns
its sequence number. The bridge uses encoder positions for both the runtime
position and encoder fields. VRPN data is displayed separately and does not
currently feed the PFCR control loop.

With Z simulated, the bridge leases only X/Y and integrates Z at commanded
speed until its target. Physical Z stays in place. The simulated axis starts
at the node's reported Z position, so the node must still expose that position.

The bridge raises each real axis's speed magnitude to at least
`max(0.1 * max_speed, min_speed)`. This addresses pulse-drive acceleration at
low starting speeds; the current implementation applies it to every real axis
in the bridge. Commands are then checked against the node profile.

The line protocol is defined in `include/rvd/bridge.hpp`: `READ`, `SEND`, `HOLD`.
`HOLD` releases the lease. Timestamp age is estimated from the node sample and
half a clock-request round trip; uncertainty adds 16 ms for coarse Windows
clocks. RVDLAB sets clock uncertainty tolerance to 0.1 s, communication timeout
to 0.8 s, and measurement lateness to 0.5 s. `--fast` cannot be used with a bridge.
The bridge acknowledgment means the node completed the command call, not that
the axis reached its target.

## Motion capture

An optional profile `mocap` object selects `driver: "vrpn"`, `host`, optional
`port` (3883) and `local_ip`, and named `trackers` with a sensor index (default 0).
Each tracker can supply `to_rail.matrix` (3x3) and `to_rail.offset` (three values).
The receiver implements VRPN TCP directly, without a compiled VRPN dependency.

State includes raw position, `quaternion_xyzw`, source time, receipt age and
validity, plus mapped rail position when configured. Defaults are 0.25 s for
`max_age_s`, 0.5 s for `max_repeat_s`, and 5 s for `reconnect_after_s`.
Bit-identical poses with advancing source timestamps become invalid after the
repeat threshold; `max_repeat_s: null` disables that check. Use probe's
`--mocap-seconds` option to change the listening interval.

The C++ `Pose` type uses w,x,y,z quaternion order. Its camera, LiDAR,
motion-capture and target-platform interfaces remain abstract; the node's VRPN
receiver and PLC axes are separate implementations.
