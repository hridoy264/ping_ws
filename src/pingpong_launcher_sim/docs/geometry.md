> Historical R5 record. This does not validate the physical R10 robot. See the [current package guide](../README.md) and [head-meter integration record](head_meter_integration.md).

# R5 model geometry and physics scope

The model uses the saved R5 front cover and the retained R2/R3/R4 printable
components as detailed **visuals**. It is a prototype for aiming, actuator commands,
and approximate free-flight launches. Mass properties, motor behaviour, wheel
contact and feed reliability are not calibrated to hardware.

## Reproducible mesh conversion

From the original repository, run:

```bash
python3 ros2_ws/src/pingpong_launcher_sim/tools/build_model.py
```

The generator uses only the Python standard library. It reads the original CAD
STL files without modifying them, writes binary STL files in **metres**, and writes
`models/pingpong_launcher/mesh_manifest.json`. The manifest records every source
path, source SHA-256, rotation, translation, triangle count and output bounds.
The shipped meshes work without Fusion or the original CAD checkout. Do not add
another `0.001` mesh scale: the SDF mesh scale is already `1 1 1`.

Fusion's body export uses the component's coordinates, not necessarily the final
assembly coordinates. The reconstruction follows these source scripts:

- `fusion/ThrowerR2/build.py`: wheel stations at 90°, 210°, 330°, centre radius
  52 mm; wheel core radius 30.5 mm, rim shoulders 31.5 mm, rubber radius 32.5 mm.
- `fusion/FeederR3/build.py`: original launch +Z becomes assembly +X; original
  radial +Y becomes assembly +Z. Feeder rotor centre is initially X = −225 mm.
- `fusion/AimingR4/build.py`: head raised to Z = 230 mm; feeder shifted rearward
  by 60 mm; head pivot X = −33 mm. Aiming supports are drawn in assembly
  coordinates; A02 head inlet retains original head-component coordinates.
- `fusion/AimingR4/repair_joints.py`: repaired yaw/pitch axis placements and limits.
- `fusion/AimingR5/update.py`: shallow front reaches 55 mm beyond the wheel plane.

R2 rear shell, R5 cover and R4 head inlet vertex conversion is
`(x,y,z) mm → (z,x,y)/1000 + (0.033,0,0)` in the head frame. R4 support parts
are divided by 1000 and translated by the negative link origin. R3 stationary
feeder parts are divided by 1000 and moved −0.06 m in X. The rotor is additionally
re-centred at its shaft, so it rotates independently instead of remaining locked
inside the original Fusion fixed group.

The old deep R4 cover, old guide, chute, cradle and print coupons are excluded.
The purchased motor/servo/shaft visuals use simplified envelopes. Printed gear
meshes are visual; motor pinions are fixed cosmetic pieces and gear meshing is
not simulated. Output yaw and pitch joints are actuated directly. The neutral
CAD hose is omitted because its shape would be false when the head turns.

## Frames and joints

All coordinates below are relative to the model at neutral. The world can place
the entire model at a bench height; the `world_fixed` joint anchors its base at
that initial placement without making the articulated model static.

| Link | Neutral origin, metres | Parent/joint |
|---|---|---|
| `base_link` | (0, 0, 0) | world / `world_fixed` |
| `yaw_link` | (−0.033, 0, 0.084) | base / `yaw_joint` |
| `head_link` | (−0.033, 0, 0.230) | yaw / `pitch_joint` |
| `wheel_1_link` | (0, 0, 0.282) | head / `wheel_1_joint` |
| `wheel_2_link` | (0, −0.0450333, 0.204) | head / `wheel_2_joint` |
| `wheel_3_link` | (0, +0.0450333, 0.204) | head / `wheel_3_joint` |
| `feeder_link` | (−0.285, 0, 0.3506) | base / `feeder_joint` |

Head axes are +X forward, +Y left, +Z up. Yaw rotates about +Z with provisional
limits ±25°. Pitch rotates about +Y, with limits −20° to +10°: a **negative pitch
command points upward**. The rotor rotates about +Z without a travel bound;
90° position increments represent its four-pocket indexing motion.

For wheel angle θ = 90°, 210°, 330°, its head-local centre is
`(0.033, 0.052 cos θ, 0.052 sin θ)` and its joint axis is
`(0, −sin θ, cos θ)`. Positive rotation makes the innermost tread velocity point
along +X for every wheel. Link axes remain aligned with the head at neutral;
mesh orientation and inertia tensors account for the three different axle axes.

The cover's exit is `(0.088,0,0)` in `head_link`; the plugin's `muzzle_pose` is
`(0.113,0,0,0,0,0)`. Thus the 20 mm radius ball starts 25 mm beyond the exit
plane, leaving a 5 mm gap from the ball's rear surface. This is a free-flight
release pose, not a simulated passage through the wheels or feeder.

## Inertial estimates and actuator assumptions

Every link has positive mass and a positive-definite rigid-body inertia at an
explicit centre of mass. These are engineering placeholders, not Fusion-derived
material measurements. The generator calculates box/cylinder inertias and rotates
wheel tensors to their actual axes. Replace them with measured or material-aware
CAD values before interpreting torques, vibration or balance.

| Link | Mass kg | Approximation | COM in link, metres |
|---|---:|---|---|
| base | 2.80 | uniform 0.58 × 0.25 × 0.59 m box | (−0.19, 0, 0.22) |
| yaw | 0.62 | uniform 0.24 × 0.345 × 0.20 m box | (0, 0.012, 0.035) |
| head excluding wheel links | 0.93 | uniform 0.16 × 0.24 × 0.24 m box | (0.005, 0, 0) |
| each wheel | 0.035 | solid cylinder, radius 0.0325 m, length 0.018 m | (0, 0, 0) |
| feeder rotor | 0.09 | solid cylinder, radius 0.086 m, length 0.0415 m | (0, 0, 0.02075) |

Native Fortress `JointPositionController` uses `use_velocity_commands=true` for
yaw, pitch and feeder. It imposes ideal position tracking with commanded speed
caps of 0.6, 0.45 and 2 rad/s respectively. Wheels use native `JointController`
in velocity mode. Joint limits include provisional effort/velocity bounds, but
these controllers do **not** reproduce current-limited motors, servo dynamics,
gear backlash or torque saturation. The wheel speed limit is 1200 rad/s, an
exploratory simulation bound rather than a hardware rating.

Reference APIs: [Fortress position controller](https://gazebosim.org/api/sim/6/classignition_1_1gazebo_1_1systems_1_1JointPositionController.html),
[velocity controller](https://gazebosim.org/api/sim/6/classignition_1_1gazebo_1_1systems_1_1JointController.html),
[joint state publisher](https://gazebosim.org/api/sim/6/classignition_1_1gazebo_1_1systems_1_1JointStatePublisher.html).

## Collision and launch approximations

Detailed threads and print features are visual meshes. All launcher collision
shapes are primitive boxes or cylinders; there is no concave mesh dependency.
The cover, rear plate, inlet and feeder pockets are approximated by convex panels
around open bores. The launch corridor is not sealed by a bounding-box collision.
Self-collision is disabled for this linked assembly; interference at extreme
poses must be evaluated separately in CAD or with more detailed collision tests.

The feeder has its real four-pocket visual mesh and independently rotating link;
its collision proxy includes a hub and four ring-shaped pockets. The pan, lid,
and basket collision geometry is incomplete. There is no loaded hopper, ball
singulation, hose deformation, hose contact or gravity-feed model. A feeder
index command and a fire command are separate operations; software may sequence
them, but the physics does not establish successful feeding.

`pingpong::BallLauncher` reads measured wheel joint speeds and approximates exit
speed as their mean × 0.0325 m × efficiency (default 0.75). It creates free-flight
balls beyond the muzzle, so internal wheel compression and tread friction are
not computed. Ball defaults are 40 mm diameter and 2.7 g; aerodynamic drag uses
Cd = 0.47 and air density 1.225 kg/m³. The initial plugin does not model
differential-wheel spin or Magnus forces. Calibrate the efficiency, drag and
contact properties against real launches before treating landing predictions
as physical validation.
