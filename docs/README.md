# RVDLAB documentation

Start with a local simulation, then choose the task below. Follow each task's
links in order; review motion configuration and stop behavior before operating
a physical platform.

- **Install and run a simulation.** Begin with the [quick start](../README.md#quick-start)
  for package requirements, installation, and a local run. Continue with
  [simulation settings](configuration.md#simulation-and-node-runs), or follow
  [Build from source](development.md#build-from-source) to use a source checkout.
- **Connect a laboratory platform.** Read [safety and stop behavior](safety.md),
  then follow [node setup](hardware-integration.md#set-up-a-node),
  [hardware profiles](hardware-integration.md#hardware-profile), and
  [control and operation](hardware-integration.md#control-and-operation).
- **Configure motion and stopping.** Start with [runtime configuration](configuration.md#runtime-configuration)
  and [command modes and coordinate frames](configuration.md#command-modes-and-frames).
  Then review [node supervision](safety.md#node-supervision) and
  [held axes and parking](safety.md#held-axes-and-parking).
- **Inspect and replay experiment records.** Follow [records and replay](configuration.md#records-and-replay)
  for saved files and offline verification, then read
  [TUI trajectory comparison](tui-trajectory-comparison.md) to interpret the
  displayed paths and intervention details.
- **Develop controllers and device adapters.** Start with [Build from source](development.md#build-from-source)
  and the [architecture](development.md#architecture) for the controller interface.
  Use [hardware integration](hardware-integration.md) for device profiles and
  bridge behavior, then run the [tests and checks](development.md#tests-and-checks).

The [numerical baseline](baseline.md) records the original measurements and
regression limits. Return to the [project overview](../README.md) for the system
architecture and motion lifecycle.
