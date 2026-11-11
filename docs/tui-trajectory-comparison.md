# TUI trajectory comparison

Autonomous Rendezvous shows these views after a run finishes:

- **Actual (white and orange ×):** the logged Hill-frame position used by the controller. Orange × marks a step where the safety filter changed the requested force, including backup intervention. It is part of the actual path, not a separate trajectory.
- **No CBF (purple dashed):** an independent model-only rollout from the first logged state. The same policy is evaluated again at each new state, with its own nominal/applied action history and no intervention feedback. It uses the deployed CWH Euler integration, actuator lag, scale and deadzone, but no CBF-QP or backup guard, rail plant, measurement noise, or tracking correction. It ends on capture, front-plane crossing, model-domain exit or timeout. This is a counterfactual, not a preplanned path or a controlled ablation of the physical rail.
- **Local detail:** select an intervention step. Cyan holds that step's raw requested force for 8 seconds; green holds the filtered force for 8 seconds. Both start from the same logged state and previous actuator output, and include actuator dynamics. White shows the actually observed following 8 seconds, when the policy and filter continued updating. The initial position is ○. The largest force correction is selected initially.

Local predictions are illustrative held-command forecasts, not certified trajectories. They do not reevaluate the policy or CBF during their horizon. In the deployed Euler scheme, force first affects velocity; the next position alone cannot show the intervention's effect. Coincident paths share terminal cells, with the actual path drawn on top.

The runtime completes normally before RVDLAB calls `rail_replay --preview EXPERIMENT_DIR MODEL_DIR NEW_OUTPUT_DIR`. This mode only reads recordings and writes comparison CSV files. It has no hardware interface and does not change the controller or its safety settings. Comparison files are stored under `experiments/comparisons/<run-id>/`; original recording checksums remain valid. Older runtimes without preview support display an unavailable message while retaining the actual trajectory.
