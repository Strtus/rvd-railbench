# Safety boundaries

PFCR's CBF-QP, backup recovery, CWH dynamics, actuator model, trained weights and
certification bounds are unchanged. Golden regression evaluates their existing
numerical contract. Algorithmic guarantees remain limited to the original
model and assumptions; they do not certify rails, drives, sensors or contact.

The independent `SafetySupervisor` enforces software operating limits: finite
measurements, measured and commanded rail travel, command speed and per-period
acceleration, tracking error, required observations, acquisition/receipt timing,
clock uncertainty, I/O failure, command/acknowledgment timeout, controller output
validity, execution deadline and E-stop requests. It does not clamp a dangerous
command and continue. A fault is latched and prevents subsequent commands.

| From | Allowed next states |
|---|---|
| INITIALIZING | READY, FAULT |
| READY | RUNNING, STOPPED, FAULT |
| RUNNING | HOLD, STOPPED, FAULT |
| HOLD | READY, STOPPED, FAULT |
| FAULT | STOPPED |
| STOPPED | none |

`fault()` preserves the first cause. There is no fault-reset transition; restart
and reinitialize after correcting the cause. The executable terminates on a
fault, docking, or maximum-step limit; it does not resume automatically from
HOLD. A maximum-step limit retains exit code 1, but is labeled `LIMIT_REACHED`
in the summary rather than mistaken for a hardware failure.

Changes to runtime validation are intentional corrections: NaN/Inf cannot bypass
comparisons; initial measurements are checked; out-of-order frames and unknown
clock mappings are rejected; absent timestamps no longer masquerade as fresh;
command acceleration and full-period overruns stop execution. `--fast` now uses
virtual time and runs timestamp checks. The old half-period warning remains
diagnostic. Receipt-time fallback and existing freshness/skew thresholds
otherwise remain. Tests exercise corrected boundaries.

Simulation motion limits are configurable physical operating constraints, not
new certified PFCR bounds. Acceleration defaults are simulation choices and
have no measured hardware rating. The bounded simulator may miss a command
deadline; tracking supervision then faults. Commands are never sent to real
hardware by CI or replay. E-stop, drive protections, PLC interlocks and watchdogs
must remain independent of this process.

Known numerical limit: original free-running Golden runs 1 and 3 can drift at
policy/safety decision boundaries despite passing the forced-action test. This
was present before refactoring. It is documented, not repaired through
reference regeneration or tolerance changes. No physical safety certification,
real-time scheduling guarantee or physical experiment success is claimed.
