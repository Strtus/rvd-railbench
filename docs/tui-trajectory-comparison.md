# TUI trajectory comparison

The run status displays signed **X / Y / Z velocity components**, with a shared `Velocity [m/s]`
header. **Hill model** uses the controller's Hill-frame velocity. **Rail encoder** divides each
physical rail-axis displacement by the elapsed encoder sample time, using the original node
timestamps in `sensors.csv`. Signs are preserved; these rows do not display velocity magnitudes.
**Encoder interval** shows the averaging interval. The sensor log is recorded even without mocap;
when a controller axis is simulated, the encoder row still uses that axis's physical encoder.
Missing, repeated or invalid samples show an unavailable value. If consecutive encoder frames
share a timestamp and position, that measurement is kept until a later timestamp arrives.
Simulator runs show `Rail encoder — (simulation)`. A finished run retains the last recorded samples, labelled
`finished · last sample`; it does not report the rail's speed after recording ends.

**Mocap raw rel.** divides each component of the change in `Target - Chaser` position by the camera
source-time interval. The components use the original motion-capture axes and physical m/s;
they are not assumed to align with the rail axes and have no Hill scaling. **Mocap interval**
shows this interval. Both trackers must be valid, have advancing
frame counters and source timestamps, and belong to the same camera frame (within 1 microsecond).
Missing, stale, invalid, repeated or unsynchronized poses show an unavailable value; recovery
requires two fresh valid pairs. The receiver's validity flag supplies the configured freshness check.
This display uses the recorded poses, not the encoder values or a separate camera velocity feed,
and does not feed the control loop.

The Hill velocity history uses three rows labelled X, Y and Z in the existing sparkline area.
Each axis has its own scale. The telemetry page uses the same three-component display.

Autonomous Rendezvous shows these views after a run finishes:

- **Actual (white and orange ×):** the logged Hill-frame position used by the controller. Orange × marks a step where the safety filter changed the requested force, including backup intervention. It is part of the actual path, not a separate trajectory.
- **No CBF (purple dashed):** an independent model-only rollout from the first logged state. The same policy is evaluated again at each new state, with its own nominal/applied action history and no intervention feedback. It uses the deployed CWH Euler integration, actuator lag, scale and deadzone, but no CBF-QP or backup guard, rail plant, measurement noise, or tracking correction. It ends on capture, front-plane crossing, model-domain exit or timeout. This is a counterfactual, not a preplanned path or a controlled ablation of the physical rail.
- **Local detail:** select an intervention step. Cyan holds that step's raw requested force for 8 seconds; green holds the filtered force for 8 seconds. Both start from the same logged state and previous actuator output, and include actuator dynamics. White shows the actually observed following 8 seconds, when the policy and filter continued updating. The initial position is ○. The largest force correction is selected initially.

Local predictions are illustrative held-command forecasts, not certified trajectories. They do not reevaluate the policy or CBF during their horizon. In the deployed Euler scheme, force first affects velocity; the next position alone cannot show the intervention's effect. Coincident paths share terminal cells, with the actual path drawn on top.

The runtime completes normally before RVDLAB calls `rail_replay --preview EXPERIMENT_DIR MODEL_DIR NEW_OUTPUT_DIR`. This mode only reads recordings and writes comparison CSV files. It has no hardware interface and does not change the controller or its safety settings. Comparison files are stored under `experiments/comparisons/<run-id>/`; original recording checksums remain valid. Older runtimes without preview support display an unavailable message while retaining the actual trajectory.
