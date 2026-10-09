# R10 head feeder, idler and drain simulation update — 9 October 2026

The physical model now includes the integrated CAD meter and manual drain. It uses68 native Fusion body exports for the new components and fixed lower pipe, with simplified collision shapes. Native mesh bounds were compared against the export manifest in neutral model coordinates: maximum bounding-box difference0.00691mm. Millimetre meshes use an explicit0.001 scale and link-origin offsets.

## Mechanism and commands

- Driven tire and passive idler: radius6mm, face Z183..189mm, centres X192/Y±25/Z186mm. The nominal unloaded gap is38mm.
- Idler carriage: prismatic axis−Y, travel0..2mm; the idler revolute joint is parented to that carriage. Both follow the aiming head.
- Motor and driven roller have separate revolute joints. `/pingpong/head_meter_velocity_cmd` is a Double/Float64 velocity inrad/s sent to both ideal controllers for their1:1 ratio. Positive velocity moves the driven contact toward+X. Zero stops the ideal controllers; it does not establish hardware brake performance.
- Lower feeder uses `/pingpong/physical_feeder_velocity_cmd`. This physical mode does not use the legacy four-pocket feed sequence or create-at-muzzle launcher.
- Drain cover and side access panel are separate installed fixed links. Hollow pipe collision panels are partitioned at the CAD Y−120.8mm/Z−8mm split planes; no solid hull fills the pipe bore. The side wall has the service opening. Automatic removal and drain dynamics are not implemented.
- Actual joint states include all four meter joints. The physical ball monitor and all100 persistent identities are preserved.

Run in the existing sourced ROS2 Humble/Fortress workspace:

```sh
ros2 launch pingpong_launcher_sim physical.launch.py gui:=false
```

The dedicated bridge includes head-meter velocity, physical lower-feeder velocity, aiming/wheel commands, measured joints and physical inventory/count. It does not start the legacy abstract launcher controller. The100-ball world remains a development scenario whose full settling/feed test has not passed.

## Verification

-36 source tests passed, including mesh closure/positive volume, exact100 unique balls with non-overlapping initial positions, absence of the abstract launcher plugin, roller dimensions/parentage, idler limits and packaged CAD visuals.
-`ign sdf -k` returned Valid. The updated package built successfully, and the new launch entry's arguments resolved.
-`ping_ws/head_meter_runtime/report.json`: unloaded full-mechanism native test passed over4.806 simulated seconds and238 observed samples. Both motor and driven roller accepted+2rad/s,−2rad/s and stop; yaw reached0.15rad, pitch0.10rad; idler stayed within its travel bounds.
-The first test incorrectly evaluated reversal before command delivery. Its report and samples remain saved as `first_run_*`. The corrected test waits for measured commanded velocity before timing each phase; it does not weaken the rotation/displacement requirements.

## Explicit remaining approximations

Spring force and compliant tire properties are unselected. The slide currently has zero spring stiffness and provisional0.02N·s/m damping. It is a passive free slide, not a qualified spring model. Tires are rigid cylinders; ordinary model friction is inherited and has not been calibrated. Belt force transmission, stall behaviour and a power-off brake are absent. Masses and inertias are estimates.

Fasteners, teeth, bearing details, optical modules and the nominal spring are detailed visuals; only ball-accessible rails/motor/treads/carriage and simplified hollow-pipe/flange geometry are contact shapes. The optical modules are not functioning beam sensors yet. The spring visual is a nominal envelope and does not deform with carriage travel. Legacy head/enclosure visuals remain approximations, including the old inlet visual without sensor cross-drills; the model is not a final whole-robot CAD rendering.

No new100-ball settling, physical priming, launch, spin or drain-emptying pass is claimed. Those require subsequent contact calibration and loaded tests.

## Ubuntu handoff

Follow the [workspace run guide](../../../README.md) for installation, build and first launch, and the [package guide](../README.md) for controls. The unloaded check uses the installed package prefix and no longer requires a `/ws` directory:

```bash
# From the workspace root, after sourcing ROS and install/setup.bash:
python3 src/pingpong_launcher_sim/tools/check_head_meter_runtime.py
```

The portable check was rerun successfully in the Ubuntu development container. This remains an unloaded joint test; it does not validate physical ball transport.
