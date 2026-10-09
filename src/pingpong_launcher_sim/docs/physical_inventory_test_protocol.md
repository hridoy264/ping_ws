# Physical 100-ball runtime protocol

`tools/physical_inventory_smoke.py` reads actual snapshots published by the
physical-ball monitor on `/pingpong/physical_ball_inventory`. It never requests
a new ball, uses the old abstract launcher count, or changes a body pose. It
requires the monitor plugin and a populated physical world. The tool does not
start a ROS controller or command the feeder by itself.

## Run an inventory settling check

After building and sourcing the Fortress workspace, run from the package directory (`cd src/pingpong_launcher_sim` from the workspace root). The generated world exists, but its full settling/feeding test has not passed. This tool starts its own server; do not separately launch the world.

```bash
export IGN_GAZEBO_RESOURCE_PATH="$(ros2 pkg prefix --share pingpong_launcher_sim)/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$(ros2 pkg prefix pingpong_launcher_sim)/lib${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"
python3 tools/physical_inventory_smoke.py \
  --world worlds/physical_100.sdf \
  --duration 30 --timeout 180 \
  --report /tmp/physical_100_inventory.json
```

The tool starts an unpaused headless Gazebo server in a unique transport
partition and terminates only its own processes afterward. Resource/plugin
paths must already resolve through the sourced workspace. Server output is
saved beside the report as `physical_100_inventory.gazebo.log`.

For an existing simulation, use `--attach` instead of `--world` and match its
`IGN_PARTITION` environment or pass `--partition NAME`. The existing simulator
is neither started nor stopped in attach mode.

## Run a feeding check

```bash
python3 tools/physical_inventory_smoke.py \
  --attach --duration 120 --timeout 600 --min-shots 100 \
  --report /tmp/physical_100_feeding.json
```

Start the physical feeder after the harness announces that it observes all
100 IDs. The controller must act on the physical mechanism and existing
inventory balls. The previous R5 spawn-on-fire plugin must be disabled in
this world. A feeder command or successful service response is not counted.

The shot target counts **new unique monitored muzzle crossings during the
measurement window**. Crossings that happened before the first complete
snapshot do not satisfy the target. If the target is not reached by the end
of the requested simulated duration, the run fails. Increase the duration to
match the intended feeding rate and setup time; wall timeout separately
allows for slower-than-real-time contact physics.

## Pass meaning

The expected default names are `inventory_ball_001` through
`inventory_ball_100`, each on its original persistent entity. Every complete
snapshot must contain the exact set, finite world positions, no duplicate
names, and a physical shot count matching the individually crossed IDs.
Model/link entity IDs must remain unchanged, not merely reuse the same names.
The harness accepts only the `physical_ball_monitor/v1` JSON schema and rejects
identity-replacement reports or a changed monitor reset epoch.
Before the first complete snapshot, a partial inventory is treated as startup;
after it, a missing entity fails immediately. Time or crossing-history resets
during the measured window fail the run.

Reports include initial/final positions, global observed minimum/maximum XYZ,
simulation duration, wall duration, sample count and new crossing IDs. Optional
`--allowed-bounds xmin ymin zmin xmax ymax zmax` rejects ball centres leaving
an explicitly permitted world region. Without that option, inventory
conservation alone cannot establish that balls stayed inside the hopper.

For a settling run, `--containment-bounds` additionally checks final whole-ball
envelopes against an explicit box, allowing 0.1 mm numerical surface tolerance.
The current R10 feeder/loading-neck/hopper bounding envelope is
`--containment-bounds -0.272 -0.1638 0.840 -0.112 0.0462 1.258`.
This catches balls remaining above the rim after settling; it does not assert
that every point in the tapered volume is open interior space. Do not impose
hopper-only containment on a feeding test whose balls should leave the hopper.

The harness also reports the closest sampled ball pair, sampled maximum
ball-to-ball penetration, and the final maximum centre displacement divided by
the last sample interval. Default `--max-penetration 0.001` fails gross sampled
overlap over 1 mm. Monitor sampling can miss shorter transients; these are
sampled diagnostics, not continuous contact certification. Low final motion
and final containment should both be reviewed before describing the load as
settled.

With the default `--min-shots 0`, a successful conservation run reports
`inventory_pass: true` and **`feeding_pass: false`**. A positive target enables
the crossing test; `feeding_pass: true` then means the requested number of
persistent balls crossed the monitor plane while the inventory checks held.
This alone does not establish the quality of each shot, complete feed-path
transit, rubber contact accuracy, spin transfer or real hardware performance.

The monitor topic is inventory-scoped. Its conservation test does not count
unrelated models or prove that another plugin did not spawn a differently
named ball. Review the physical world/plugin configuration as part of the run.

## Required staged runs

1. Empty mechanism: verify yaw/pitch motion and collision corridor.
2. One persistent ball: establish correct passage and monitor crossing.
3. Small batch: observe singulation, ball-to-ball contacts and stalls.
4. 100-ball settling: exact count and finite poses; inspect containment.
5. 100-ball feeding: require 100 new muzzle crossings, retaining IDs afterward.
6. Repeat for multiple initial packing seeds and supported head positions;
   record jam, double-feed and transit observations separately.

Do not report a stage as passed because a prior stage or source-only test
passed. Save the generated input world, geometry revision, monitor/controller
configuration and JSON/log files with each runtime result.

## Work checkpoint — 7 October 2026

- The initial full mesh collision model retained all 100 observed identities,
  but timed out after 180 wall seconds with only 1.002 simulated seconds. The
  saved `ping_ws/physical_100_settling_initial.json` is a **failed runtime test**,
  not a settling or feeding pass.
- The optimized generator replaces pipe contacts with circumscribed primitive
  box panels, the hopper taper with four sloping panels, and the pan floor with
  a cylinder/box union. Detailed proxy surfaces remain visuals. The rotor keeps
  78 individually convex contact prisms; its six pockets remain open.
- A neutral-pose static check of 505 centerline samples found a minimum pipe
  diameter clearance of approximately 44.97 mm, above the 44 mm collision-bore
  requirement. This samples geometry only; it does not establish moving-joint
  passage or successful contact dynamics. The retained head inlet is a separate
  43 mm collision approximation and wheel compression remains uncalibrated.
- The optimized source/world are saved. Its native SDF check and at least a
  five-second settling/retention run still need repeating when Docker is
  resumed. Do not carry the previous geometry's validation forward.
