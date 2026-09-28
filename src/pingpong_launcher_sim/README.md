# R5 launcher — ROS 2 Humble / Gazebo Fortress

This package targets **Ubuntu 22.04, ROS 2 Humble, and Gazebo Fortress (Ignition Gazebo 6)**. It contains the R5 CAD visuals, six independently moving joints, ROS command topics, feeder sequencing, and a ball-launch system. It uses native Gazebo controllers and `ros_gz_bridge`; `ros2_control` is not required.

## Install and launch on Ubuntu 22.04

Install ROS 2 Humble using the [official Ubuntu installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) first. Fortress is the [recommended Gazebo pairing for Humble](https://gazebosim.org/docs/fortress/ros_installation/). This package does not use Gazebo Classic 11 or Harmonic.

Extract `PingPong_ROS2_Humble_Fortress.zip` into your Ubuntu home directory. The
archive contains `ros2_ws/src` only, so the build uses your machine's own paths
and architecture. Then:

```bash
cd ~/ros2_ws
bash src/pingpong_launcher_sim/scripts/install_ubuntu_dependencies.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pingpong_launcher_sim
source install/setup.bash
ros2 launch pingpong_launcher_sim sim.launch.py
```

The scene contains the launcher on a bench, a table, and a simple rigid net. The wheels start stopped. For a server without a display:

```bash
ros2 launch pingpong_launcher_sim sim.launch.py gui:=false
```

An optional starting pose and speed can be supplied with `yaw_deg:=10.0 elevation_deg:=10.0 initial_rpm:=2500.0`. No ball fires automatically. Start only one instance at a time; this first version uses fixed topic and model names.

## Aim, spin up, and fire

In another terminal, source the workspace again:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# x = yaw degrees, y = upward elevation degrees; z is unused.
ros2 topic pub --once /pingpong/aim geometry_msgs/msg/Vector3 \
  '{x: 10.0, y: 10.0, z: 0.0}'

# x/y/z = the three wheel speeds, in RPM.
ros2 topic pub --once /pingpong/wheel_rpm geometry_msgs/msg/Vector3 \
  '{x: 2500.0, y: 2500.0, z: 2500.0}'

# Wait for the wheels to spin up, then request one shot.
ros2 service call /pingpong/fire std_srvs/srv/Trigger '{}'

# Observe actual launches and joint feedback.
ros2 topic echo /pingpong/shot_count
# In a separate terminal:
ros2 topic echo /pingpong/joint_states
```

A successful fire service response means the request was accepted. The controller advances the feeder by 90 degrees, waits for measured position to settle, then requests a ball. The ball system increments `shot_count` only after confirming the ball received its launch velocity. `/pingpong/status` and `/pingpong/ball_status` explain rejection or cancellation. Requests are rejected while another feeder index is pending, when joint feedback is stale, or while any wheel is stopped.

```bash
ros2 service call /pingpong/stop std_srvs/srv/Trigger '{}'
```

Stop cancels a pending feeder sequence and commands zero wheel speed; it holds the feeder at its current position. It does not remove balls already in flight.

## Interfaces and units

| ROS interface | Type | Meaning |
|---|---|---|
| `/pingpong/aim` | `geometry_msgs/msg/Vector3` | x yaw: −25..+25°, y elevation: −10..+20° |
| `/pingpong/wheel_rpm` | `geometry_msgs/msg/Vector3` | x/y/z wheel 1/2/3: 0..6000 RPM |
| `/pingpong/fire` | `std_srvs/srv/Trigger` | One feeder index and launch request |
| `/pingpong/stop` | `std_srvs/srv/Trigger` | Stop wheel commands, cancel queued shot |
| `/pingpong/joint_states` | `sensor_msgs/msg/JointState` | Measured radians and radians/second |
| `/pingpong/shot_count` | `std_msgs/msg/UInt32` | Confirmed launch count since simulation start/reset |
| `/pingpong/status` | `std_msgs/msg/String` | ROS sequencing status |
| `/pingpong/ball_status` | `std_msgs/msg/String` | Ball-system status |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time |
| `/pingpong/dynamic_poses` | `tf2_msgs/msg/TFMessage` | Gazebo entity poses, including launched balls |

Coordinates are meters: +X forward, +Y left, +Z up. Positive yaw turns left. The underlying pitch joint rotates about +Y, so **positive elevation means negative pitch joint angle**. Each wheel axis is oriented so positive speed would drive the ball forward. Mesh vertices have already been converted from millimeters to meters.

The raw `*_cmd` topics and `/pingpong/fire_request` are bridge interfaces used by the controller. Do not publish to them simultaneously with the controller: it republishes its setpoints at 20 Hz. A separate ROS domain can isolate this simulation from other ROS work (`export ROS_DOMAIN_ID=42` before every relevant terminal).

## Simulation fidelity

The aiming and wheel/feeder motions are simulated joints. CAD visuals retain the R5 shallow 90 mm outlet cover and retained R4/R3 parts; collisions are simplified for stability. Masses, inertias, controller gains, motor limits, contact coefficients and aerodynamic values are **initial estimates**, not measurements. The 6000 RPM command limit is a simulation setting, not a physical operating rating.

The ball model deliberately starts a 40 mm, 2.7 g sphere just beyond the outlet after feeder indexing. Its initial forward speed is:

```text
speed = efficiency × 0.0325 m × mean(measured wheel angular velocities)
default efficiency = 0.75
```

Gazebo then simulates gravity and collisions, with approximate quadratic air drag. Different wheel speeds currently affect the mean launch speed only. **Wheel contact, rubber compression, ball spin/Magnus force, basket singulation, and ball transport through the flexible hose are not modeled.** The displayed feeder rotates, but there is no inventory of balls passing through its pockets. This package can test aiming, commands, sequencing and approximate trajectories; it cannot establish jam-free feeding or accurate spin shots. Table/net contacts are coarse approximations too.

Tune `<plugin name="pingpong::BallLauncher">` in `models/pingpong_launcher/model.sdf` to change efficiency, drag, ball mass/radius, maximum active balls and lifetime. The default sphere spawn pose is 25 mm beyond the cover exit; changing ball radius also requires checking this clearance. Rebuild after editing installed assets, or use `--symlink-install` as above. See [geometry notes](docs/geometry.md) for CAD transforms and collision approximations.

## Tests

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select pingpong_launcher_sim --event-handlers console_direct+
colcon test-result --verbose

# Starts and stops its own headless Gazebo instance:
python3 src/pingpong_launcher_sim/test/runtime_smoke.py
```

The runtime test checks feedback, aiming, all wheel velocities, five successive
shots (crossing a full feeder revolution), sustained ball movement, and stopping
with a pending shot. Run it with no other simulation instance using the same ROS
domain. See `docs/validation.md` for the recorded results and limitations.

## Troubleshooting

- `ign: command not found`: install `ignition-fortress` / `ros-humble-ros-gz` using the dependency script. Fortress uses the `ign gazebo` command.
- Model or plugin cannot be found: source this workspace and use `sim.launch.py`; it sets `IGN_GAZEBO_RESOURCE_PATH` and `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` from the installed package.
- GUI unavailable in SSH, Docker, or a VM: use `gui:=false`. On a local Ubuntu desktop, confirm OpenGL works; `LIBGL_ALWAYS_SOFTWARE=1` can help with a VM graphics driver.
- Fire rejected: unpause Gazebo, check joint feedback, and let all three wheels spin up. The controller operates in simulation time.
- `/clock` exists but no joints: check the launch output for missing physics or controller plugins, and verify you installed Fortress rather than a different Gazebo release.

## Reproducibility and source assets

The delivered meshes are self-contained; Fusion and the original workspace paths are not needed to build or run. The optional `tools/build_model.py` generator reads the original project CAD exports when regenerating model geometry. Original Fusion files are not modified. Imported reference CAD and CAD-derived geometry retain their original ownership; the package is intended for this project, and does not grant a redistribution license for third-party reference models.
