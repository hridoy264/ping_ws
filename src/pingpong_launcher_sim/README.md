# PingPong R10 physical robot simulation

Current entry point: **`physical.launch.py`**. It loads `worlds/physical_100.sdf`, which includes `models/pingpong_r10/model.sdf` and 100 persistent 40 mm / 2.7 g balls. The model includes the hollow aiming route, powered head meter, sliding idler, installed drain cover and side access panel. Models and CAD-exported meshes are included; Fusion is not required to run them.

This is a development simulation. An unloaded mechanism test passes; full 100-ball settling, feeding and launching do not yet have a passing validation. Spring force, rubber compliance and launch spin remain uncalibrated. It is not ready to predict hardware performance.

## Requirements and first build

Use Ubuntu 22.04, ROS 2 Humble and Gazebo Fortress (Ignition Gazebo 6). The [official Gazebo compatibility guide](https://gazebosim.org/docs/fortress/ros_installation/) lists Humble/Fortress as the recommended pairing. Other installed ROS versions need a separate port; do not source Jazzy alongside Humble for these commands.

The examples use `~/ping_ws` as the workspace root, with this package at `~/ping_ws/src/pingpong_launcher_sim`. Adjust only the workspace path if yours differs.

```bash
cd ~/ping_ws
bash src/pingpong_launcher_sim/scripts/install_ubuntu_dependencies.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pingpong_launcher_sim
source install/setup.bash
```

The installer expects Humble already installed and uses sudo/apt for dependencies. No Docker is needed on Ubuntu. The Git repository must include the entire source package, not only an SDF or launch file.

## Run and inspect

First run the self-contained unloaded test:

```bash
python3 src/pingpong_launcher_sim/tools/check_head_meter_runtime.py
```

It starts/stops its own server. `"pass": true` verifies measured forward/reverse/stop motion, aiming and idler limits. Results go to `~/ping_ws/head_meter_runtime/`; an optional `--output-dir /path/to/results` changes that location.

Then launch the experimental full scene:

```bash
ros2 launch pingpong_launcher_sim physical.launch.py
# Without a display, use this instead:
# ros2 launch pingpong_launcher_sim physical.launch.py gui:=false
```

The launch starts Gazebo and the physical ROS/Gazebo bridge. It does not start the legacy launcher controller. Wheels and feeder controllers initially command zero. Pause/play is available in the Gazebo window. Full-pile physics may run substantially slower than real time; wait for feedback instead of assuming wall-clock time equals simulation time. Ctrl+C stops the launch.

Use one interactive instance at a time. The automated joint test isolates its Gazebo transport partition. For separate manual sessions, set both `ROS_DOMAIN_ID` and `IGN_PARTITION` consistently in their relevant terminals.

## Commands from a second terminal

Source the same workspace in every terminal:

```bash
cd ~/ping_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

All command topics below use `std_msgs/msg/Float64`. Angles are **radians** and speeds are **radians per second**, not degrees or RPM.

```bash
# Aim: approximately +8.6 degrees yaw and +5.7 degrees upward pitch.
ros2 topic pub --once /pingpong/yaw_cmd std_msgs/msg/Float64 '{data: 0.15}'
ros2 topic pub --once /pingpong/pitch_cmd std_msgs/msg/Float64 '{data: 0.10}'

# Turn the head-transfer roller/motor at 2 rad/s, then stop it.
ros2 topic pub --once /pingpong/head_meter_velocity_cmd std_msgs/msg/Float64 '{data: 2.0}'
ros2 topic pub --once /pingpong/head_meter_velocity_cmd std_msgs/msg/Float64 '{data: 0.0}'
```

Publish those last two commands separately to observe motion between them. Negative head-meter velocity reverses it. This command tests mechanism motion; it is not a one-ball firing command. There is currently no automatic physical-mode prime/fire/jam-recovery service.

| Topic | Meaning |
|---|---|
| `/pingpong/yaw_cmd` |−0.4363..+0.4363rad (±25°); positive turns forward axis toward−Y |
| `/pingpong/pitch_cmd` |−0.3491..+0.5236rad (−20..+30°); positive raises muzzle |
| `/pingpong/head_meter_velocity_cmd` | Motor and transfer roller, ideal 1:1 velocity; positive feeds toward+X |
| `/pingpong/physical_feeder_velocity_cmd` | Lower six-pocket feeder velocity; priming trial uses 0.3 rad/s |
| `/pingpong/wheel_1_cmd`, `/pingpong/wheel_2_cmd`, `/pingpong/wheel_3_cmd` | Individual launcher-wheel velocities; contact/launch not calibrated |

The idler is passive and has 0..2 mm outward travel; it has no command topic. Its spring stiffness is currently zero because hardware force has not been selected. The modeled drain cover and access panel remain installed/fixed; no automatic drain command exists.

For a deliberate lower-feeder experiment, start slowly and observe the actual balls. This is not a validated batch recipe:

```bash
ros2 topic pub --once /pingpong/physical_feeder_velocity_cmd std_msgs/msg/Float64 '{data: 0.3}'
```

## Feedback and stopping

Each echo command runs until Ctrl+C; use separate terminals if needed:

```bash
ros2 topic echo /pingpong/joint_states
ros2 topic echo /pingpong/physical_ball_inventory
ros2 topic echo /pingpong/physical_shot_count
```

Joint feedback includes the driven roller, motor, idler rotation and idler slide. Inventory is a JSON string containing physical ball identities and observed accounting. A zero shot count may simply mean no ball passed through the muzzle; 100 entities being present is not a feed pass.

To stop commanded rotating actuators while keeping Gazebo open:

```bash
ros2 topic pub --once /pingpong/physical_feeder_velocity_cmd std_msgs/msg/Float64 '{data: 0.0}'
ros2 topic pub --once /pingpong/head_meter_velocity_cmd std_msgs/msg/Float64 '{data: 0.0}'
ros2 topic pub --once /pingpong/wheel_1_cmd std_msgs/msg/Float64 '{data: 0.0}'
ros2 topic pub --once /pingpong/wheel_2_cmd std_msgs/msg/Float64 '{data: 0.0}'
ros2 topic pub --once /pingpong/wheel_3_cmd std_msgs/msg/Float64 '{data: 0.0}'
```

Commands take effect as simulation advances; they do not freeze existing balls. Yaw/pitch retain their setpoints. For a fresh 100-ball load, stop the launch with Ctrl+C and launch again. Do not assume resetting the shot counter reloads inventory.

## Tests and rebuilding

```bash
cd ~/ping_ws
colcon test --packages-select pingpong_launcher_sim --event-handlers console_direct+
colcon test-result --verbose
python3 -m unittest discover -s src/pingpong_launcher_sim/test -p 'test_*.py'
```

The source suite includes both current physical-model invariants and legacy tests. After source changes or `git pull`, run `colcon build --symlink-install --packages-select pingpong_launcher_sim` and source `install/setup.bash` again.

Normally use the committed generated models. Developers can regenerate the R10 model/world from packaged inputs:

```bash
python3 src/pingpong_launcher_sim/tools/build_physical_model.py
```

This overwrites the generated R10 SDF, collision meshes, generation manifest and seeded 100-ball world. It does not modify Fusion. New CAD exports require the separate Fusion development workflow; simply running the generator does not export changed CAD.

## Troubleshooting

- `ign` missing: run the dependency installer. Fortress uses `ign gazebo`; this package does not use Gazebo Classic's `gazebo` command.
- Package not found: build from the workspace containing `src/` and source that workspace's `install/setup.bash` in this terminal.
- Missing model/plugin: use `physical.launch.py`, which sets installed resource/plugin paths. Keep all mesh files in Git and rebuild on Ubuntu; do not reuse Mac build outputs.
- Wrong ROS version: confirm `echo "$ROS_DISTRO"` prints `humble` after sourcing `/opt/ros/humble/setup.bash` in a fresh terminal.
- No GUI over SSH/Docker: use `gui:=false`. For a local VM graphics issue, try `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch pingpong_launcher_sim physical.launch.py`; this fallback has not been validated on your machine.
- Extremely slow pile: the 100-ball contact/settling problem is still open. Use the unloaded test to distinguish installation/joint problems from loaded physics; do not interpret a slow pile as successful feeding.
- `/pingpong/fire` or `/pingpong/aim` missing: those belong to the legacy mode. Use the physical topics above.

## Records and legacy mode

- [Current head-meter/drain integration and limitations](docs/head_meter_integration.md)
- [Physical inventory test protocol](docs/physical_inventory_test_protocol.md)
- [Physical monitor protocol](docs/physical_ball_monitor.md)
- [Historical R5 instructions](docs/legacy_r5_guide.md): `sim.launch.py`/`training.sdf`, with an abstract create-at-muzzle ball system. Its firing demonstrations do not validate physical R10 feeding.

Imported reference CAD retains its original ownership; the source package does not grant redistribution rights for third-party reference models. See `NOTICE` and `LICENSE`.
