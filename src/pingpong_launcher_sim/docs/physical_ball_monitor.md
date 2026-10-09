# Physical ball monitor

`PhysicalBallMonitor` observes persistent ball models in Gazebo Fortress. It reads
native entity poses in `PostUpdate`, after physics, and publishes measured muzzle
crossings and the ball inventory. It never creates or deletes entities, changes
poses or velocities, or applies forces. A Linux Fortress 6.18 build succeeds;
monitor runtime validation and full-robot feeding validation are separate checks.

Use this monitor with a physical feeding model and the 100 persistent models
`inventory_ball_001` through `inventory_ball_100`, each containing `ball_link`.
The legacy `BallLauncher` plugin creates balls at the muzzle and applies launch
commands; disable it for physical feeding validation. A monitor alone cannot
establish that other systems avoid spawning, teleporting, or applying artificial
launch impulses.

## Build integration

The existing package already finds `ignition-gazebo6`, `ignition-plugin1` with
`register`, and `ignition-transport11`. Add this independent target to
`CMakeLists.txt`:

```cmake
add_library(pingpong_physical_ball_monitor SHARED src/physical_ball_monitor.cpp)
target_link_libraries(pingpong_physical_ball_monitor PRIVATE
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::register
  ignition-transport11::core)
target_compile_options(pingpong_physical_ball_monitor PRIVATE
  -Wall -Wextra -Wpedantic)
install(TARGETS pingpong_physical_ball_monitor LIBRARY DESTINATION lib)
```

Build in the Linux ROS / Fortress environment:

```sh
cd /path/to/ping_ws
colcon build --packages-select pingpong_launcher_sim
source install/setup.bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$PWD/install/pingpong_launcher_sim/lib:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}"
```

This document supplies integration instructions; writing this source does not
confirm that compilation or runtime validation has occurred.

## Model configuration

Attach the plugin to the launcher model. `head_link`, `muzzle_pose`, and
`aperture_radius` are required because they must describe the physical CAD.
Coordinates use metres and angles use radians. `muzzle_pose` is the full pose of
the mouth frame relative to the head link, with order `x y z roll pitch yaw`.
The mouth frame's +X points out of the launcher; the aperture is a circle centred
at its Y=0, Z=0. `muzzle_plane_x` offsets the crossing plane within that frame.

For the revised head datums, the head origin is the pitch centre
`(0.100, 0, 0.085)` and the neutral mouth is at `(0.260, 0, 0.185)` in the same
model frame. Their head-local offset is `0.160 0 0.100 0 0 0`. Confirm those
datums against the generated SDF, especially if the head frame changes.

```xml
<plugin filename="libpingpong_physical_ball_monitor.so"
        name="pingpong::PhysicalBallMonitor">
  <head_link>head_link</head_link>
  <muzzle_pose>0.160 0 0.100 0 0 0</muzzle_pose>
  <!-- Example 50 mm clear diameter; replace with the actual mouth radius. -->
  <aperture_radius>0.025</aperture_radius>
  <ball_name_prefix>inventory_ball_</ball_name_prefix>
  <ball_link>ball_link</ball_link>
  <expected_ball_count>100</expected_ball_count>
</plugin>
```

| Parameter | Default | Meaning |
| --- | --- | --- |
| `head_link` | required | Direct launcher link defining the moving head frame. |
| `muzzle_pose` | required | Six-value mouth pose relative to that link. |
| `aperture_radius` | required | Physical clear aperture radius, greater than ball radius. |
| `muzzle_plane_x` | `0` | Mouth-frame X coordinate of the crossing plane. |
| `ball_name_prefix` | `inventory_ball_` | Prefix of world-child ball model names. |
| `ball_link` | `ball_link` | Link whose origin is the ball centre. |
| `ball_radius` | `0.020` | Radius used for whole-ball aperture clearance. |
| `expected_ball_count` | `100` | Inventory size to report; does not create balls. |
| `arming_distance` | `0.002` | Required observed distance upstream of the plane before counting. |
| `min_forward_speed` | `0.01` | Minimum forward speed in metres/second. |
| `max_ball_speed` | `100` | Continuity guard; faster sampled segments are discarded. |
| `max_sample_gap` | `0.05` | Maximum simulation-time gap between crossing samples, in seconds. |
| `publish_rate` | `5` | Periodic inventory frequency in simulation time, in Hz. |
| `shot_count_topic` | `/pingpong/physical_shot_count` | `ignition.msgs.UInt32`. |
| `inventory_topic` | `/pingpong/physical_ball_inventory` | `ignition.msgs.StringMsg` containing JSON. |
| `reset_topic` | `/pingpong/physical_monitor/reset` | `ignition.msgs.Boolean`; only true requests a reset. |
| `contact_topic` | empty | Optional existing `ignition.msgs.Contacts` stream. |

The monitor discovers only direct world-child models with the chosen prefix.
Nested balls require a different discovery contract. A missing head link is
reported through `head_available: false`; it prevents shot counting. Invalid
required configuration prevents the plugin from advertising its interfaces.

## What counts as a shot

Each model name is a persistent identity and can count once per reset. A ball
must first be observed at least `arming_distance` upstream. Two consecutive,
unpaused, finite samples must then cross from X <= `muzzle_plane_x` to X > that
plane. Both the mouth-relative forward motion and the ball's world-space motion
projected along the current mouth axis must exceed `min_forward_speed`. This
avoids counting a stationary ball merely because the head moved past it.

The monitor interpolates the centre at the crossing plane. Its radial distance
must be no greater than `aperture_radius - ball_radius`, so the whole ball clears
the circular aperture. A ball first observed downstream does not count. A later
bounce through the mouth cannot count the same identity again. Samples across a
pause, missing pose, reset, excessive sampling gap, or detected entity replacement
do not create a crossing. Excessive apparent world speed is discarded and
reported as `discontinuous_samples`.

This is a pose-based observation of actual ball entities, not a claim about the
force that accelerated them. The continuity guard cannot rule out every external
pose command. Physical feeding and wheel transfer still need contact evidence
and a review of the active simulation systems.

## Inventory JSON and readiness

The `StringMsg.data` field contains one JSON object with schema
`physical_ball_monitor/v1`. It is emitted on the first `PostUpdate`, every
`1 / publish_rate` seconds of simulation time, and immediately after a shot or
accounting reset. Paused simulation does not advance the periodic schedule.

Top-level fields:

| Fields | Meaning |
| --- | --- |
| `sim_time`, `paused`, `reset_epoch` | Simulation seconds, pause state, and accounting epoch. |
| `shot_count`, `count_semantics` | Cumulative number of unique accepted crossings in the current epoch. |
| `expected_ball_count`, `present_count`, `ever_seen_count` | Configured target; present unique names; all names observed since reset. |
| `missing_seen_count`, `unaccounted_expected_count` | Previously seen identities currently absent; positive deficit against target size. |
| `counted_present_count`, `unavailable_pose_count` | Present counted identities; present identities lacking a finite world pose. |
| `duplicate_identity_count`, `identity_replacement_count` | Duplicate current model names; changed model/link entities for a tracked name. |
| `inventory_matches_expected` | Present count equals target, no missing seen names, no unavailable world poses, and no duplicate names. |
| `head_available`, `outside_aperture_crossings`, `discontinuous_samples`, `count_overflow` | Geometry readiness and observation diagnostics. |
| `phases` | Counts by current phase. |
| `balls` | Persistent records in sorted model-name order. |

`inventory_matches_expected` checks current inventory availability. It is not a
conservation or feeding verdict: a runtime harness must also require the same
100 identities and entity IDs across samples, zero replacements, finite poses,
a stable epoch, and observed crossings for its requested shot target.

Each ball record contains:

```json
{
  "identity": "inventory_ball_001",
  "model_entity": 123,
  "link_entity": 124,
  "present": true,
  "phase": "upstream",
  "counted": false,
  "armed": true,
  "last_seen_sim_time": 0.25,
  "world_position": [0.1, 0.0, 0.3],
  "muzzle_local_position": [-0.16, 0.0, 0.115],
  "crossing_sim_time": null,
  "crossing_muzzle_local_position": null,
  "contact_pair_observations": 0
}
```

Phases are `upstream`, `downstream_uncounted`, `counted`, `missing`,
`duplicate_identity`, `missing_link`, `invalid_pose`, or `head_unavailable`.
`world_position` is null when the ball is absent or lacks a valid pose;
`muzzle_local_position` is also null when the mouth frame is unavailable. A
counted ball retains its crossing time and crossing point even when it later
becomes missing. Missing records remain in the array until an accounting reset.

Publishing true on the reset topic clears tracking, count, deduplication and
contact counters at the next `PostUpdate`; it does not change the world. A
simulation-time rewind has the same effect. Both increment `reset_epoch`, and
the first post-reset sample establishes a baseline without a fabricated shot.
Restarting the plugin begins a new instance with epoch zero.

## Contact telemetry

`contact_topic` subscribes to an existing native Contacts stream; it does not
enable contact reporting. The JSON includes `contact_telemetry_configured`,
`contact_messages_received`, `contact_pairs_received`,
`ball_contact_pairs_received`, and per-ball `contact_pair_observations`.
Ball matching uses prefixed model-name tokens in fully scoped collision names.
Unscoped or differently named collisions cannot be attributed to an identity.

These counters record received contact-pair observations, not unique impacts.
Repeated contacts on successive steps can increment them repeatedly. The monitor
always reports `contact_coverage_verified: false`: subscribing successfully or
receiving some contacts cannot prove that the stream covers every collision.
The native contact sensor/physics configuration and expected collision names
must be checked separately before asserting feeder or wheel contact transfer.

## Native smoke checks

The focused monitor test builds disposable, isolated one-ball worlds and records
the native JSON observations. Its six cases cover a gravity drop through a fixed
mouth, the reversed mouth, a passive moving mouth with a falling ball, a mouth
passing a stationary ball, repeated passive-pendulum crossings, and absent
inventory. It sends only WorldControl to start simulation; no launch, velocity,
force, teleport or entity-creation commands are used.

```sh
cd /path/to/ping_ws
source install/setup.bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$PWD/install/pingpong_launcher_sim/lib:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}"
python3 src/pingpong_launcher_sim/tools/test_physical_monitor.py \
  --report physical_monitor_native_report.json
```

Each case receives a fresh `IGN_PARTITION`; the fixture SDF, simulator log and
observations remain beside the report in its `_artifacts` directory. The report
checks conserved native entity IDs, finite positions, individually observed
forward/reverse plane crossings, count deduplication and missing-inventory
reporting. These fixtures validate the monitor and establish no robot feeding,
contact-transfer or 100-ball throughput result.

The current source compiled against Fortress 6.18 and all six fixtures passed.
The fixed and moving mouths each counted one falling ball; reversed motion and a
mouth passing a stationary ball counted zero. The pendulum produced five forward
and four reverse plane crossings while retaining one counted identity. Absent
inventory remained incomplete. The recorded run is
`ping_ws/physical_monitor_native_report.json`, with native observations in the
adjacent artifacts directory.

Observe the transport topics and request an accounting-only reset:

```sh
ign topic -t /pingpong/physical_ball_inventory -e
ign topic -t /pingpong/physical_shot_count -e
ign topic -t /pingpong/physical_monitor/reset -m ignition.msgs.Boolean -p 'data: true'
```

1. Start the physical world with exactly 100 matching models. Require all 100
   persistent names and native model/link IDs, finite world poses, no missing
   records, duplicates or replacements, and a stable epoch over the run.
2. Check neutral and moving head poses against the mouth-frame coordinates.
   Require `head_available` and finite local ball positions. A stationary ball
   swept by head motion must not count.
3. Observe an upstream physical ball crossing the clear mouth; its identity must
   become `counted` and remain present downstream. Verify the reported crossing
   point is on the configured plane and within whole-ball clearance.
4. Check first-observed downstream balls, backward crossings, off-aperture
   crossings, repeated crossings, pauses and resets. They must obey the rules
   above and never produce duplicate identity counts.
5. For feeding validation, require the requested number of newly counted
   identities plus retained inventory and native contact evidence. A
   conservation-only run with zero required shots establishes no feeding result.

The API contract is based on Fortress's read-only
[`ISystemPostUpdate`](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1ISystemPostUpdate.html),
the const enumeration methods in
[`EntityComponentManager`](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1EntityComponentManager.html),
and [`worldPose`](https://gazebosim.org/api/gazebo/6/namespaceignition_1_1gazebo.html).
The Contacts callback follows the official
[`Contact` message definition](https://raw.githubusercontent.com/gazebosim/gz-msgs/ign-msgs8/proto/ignition/msgs/contact.proto),
where collision endpoints are entity messages.
