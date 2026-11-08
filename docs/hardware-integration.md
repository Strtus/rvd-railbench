# Hardware integration contract

**No physical device driver is implemented.** `--lab` is rejected during option
validation, before policy execution or device creation. CMake refuses a
hardware-enabled build. There is no placeholder driver returning fabricated
success. No automated test connects to a rail, sensor or target platform.

Implemented software: `RailIO` contract and simulated implementation;
`MotionCaptureInput`, `CameraInput`, `LidarInput`, `TargetPlatform` and
`HardwareSafetyInput` abstract contracts. Camera/point-cloud buffers have shared
immutable ownership. Sensor frame/time tags are explicit. These contracts do
not imply support for any manufacturer, SDK, transport or acquisition pipeline.

Before enabling hardware, supply and verify:

1. Drive/PLC SDK or documented protocol, axis addressing, units, signs, zeroes,
   homing procedure, soft/hard stops and measured acceleration/braking limits.
2. Bounded read/send operations, connection-loss behavior, sequence numbers,
   command scheduling, acknowledgment meaning and cancellation/stop semantics.
3. Measured rail feedback (not echoed commands), calibrated mocap-to-rail and
   sensor/target extrinsics, quaternion convention, sensor formats and rates.
4. Device timestamp units, wraparound, clock drift/offset calibration and an
   independently justified uncertainty bound. Minimum latency is not clock sync.
5. Independent E-stop, PLC safety interlocks, drive protections and a watchdog
   that halts motion even if the host process blocks, crashes or loses power.
6. Hardware-in-the-loop commissioning and an approved operating envelope,
   followed by controlled physical validation with a local operator.

`RailIO::now_s()` and `at()` share a monotonic epoch. All validated `t_*` fields
must be mapped to that epoch. Raw sensor/PLC timestamps are provenance, never
silently treated as host time. `t_acquired` is capture time; `t_received` is
host receipt; control start/end surround controller execution; `t_sent` is
command transmission; `t_plc_ack` is device acknowledgment mapped to host time.
NaN means missing. Unknown synchronization or excessive uncertainty is rejected.
`Measurement::encoder_pos` carries the axis encoders in rail metres, read at
`t_encoder`; leave it NaN when encoders are not read. `Command::rail_reference`
is where the rail should be at `t_arrive`. In `arrive` mode it equals
`rail_target`; in `lead` mode `rail_target` is a stop point beyond the period
and `rail_velocity` carries the motion, so the device must accept a new target
and speed while moving without first stopping.
Acquisition may fall back to receipt time only when receipt exists; a missing
receipt is rejected by communication supervision. External `ClockOffset` is
explicitly an estimate with caller-provided uncertainty, not a sync service.

Device implementations must respect deadlines without unbounded blocking and
implement `hold()`/controlled stop as nonthrowing requests. The application
scope guard requests hold on all exits, but cannot force a disconnected drive
to stop. Hardware acknowledgment must be explicitly required by a future
hardware adapter; simulation never invents one. Unknown hardware safety status
is unsafe by default. A future backend must feed and validate that status
before startup and before every command. Software contracts are not physical
safety certification.
