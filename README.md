# PingPong R10 — ROS 2 / Gazebo workspace

This is the main simulation workspace. Push this folder's `src/`, `README.md` and `.gitignore` to GitHub. Keep every subfolder inside `src/pingpong_launcher_sim`, including meshes, worlds, source, launch, config, scripts and tools. Do not upload `build/`, `install/`, `log/` or generated runtime output. Fusion is not required on Ubuntu.

**Target: Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress.** This package is not configured for ROS 2 Jazzy/Harmonic or Gazebo Classic. ROS 2 alone does not install all required build dependencies. Humble/Fortress is a supported pairing in the [official Gazebo compatibility guide](https://gazebosim.org/docs/fortress/ros_installation/).

## 1. Clone on Ubuntu

These commands assume the GitHub repository root contains `src/pingpong_launcher_sim/package.xml`. Replace `YOUR_GITHUB_REPOSITORY_URL` with your actual repository URL. If already cloned, enter that workspace instead of cloning again.

```bash
git clone YOUR_GITHUB_REPOSITORY_URL ~/ping_ws
cd ~/ping_ws
```

If you pushed the whole `PingPong` project instead, enter its `ping_ws` subfolder. Run all commands below from the folder that contains `src/`.

## 2. Install dependencies and build

Install [ROS 2 Humble for Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) first if it is not already installed. With Humble installed:

```bash
cd ~/ping_ws
bash src/pingpong_launcher_sim/scripts/install_ubuntu_dependencies.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pingpong_launcher_sim
source install/setup.bash
```

Build on Ubuntu itself. Do not copy the development Mac's `build/` or `install/` directories. The dependency script verifies Ubuntu 22.04/Humble and installs the bridge, Fortress development libraries and build/test tools through apt.

## 3. Check the mechanism first

This short automated test starts its own headless Gazebo instance, commands the head feeder forward/reverse/stop and exercises aiming. It removes balls only from its temporary diagnostic world; the main world keeps 100 balls.

```bash
python3 src/pingpong_launcher_sim/tools/check_head_meter_runtime.py
```

Look for `"pass": true`. Results are saved in `head_meter_runtime/report.json`. It is an unloaded mechanism check, not a feeding test.

## 4. Open the physical robot and 100-ball scene

```bash
ros2 launch pingpong_launcher_sim physical.launch.py
```

On an Ubuntu desktop this requests the Gazebo 3D window. For SSH or a machine without a graphical display:

```bash
ros2 launch pingpong_launcher_sim physical.launch.py gui:=false
```

Press **Ctrl+C in the launch terminal** to stop. This starts an experimental physical world: 100 unique balls are present, but full-pile settling is currently slow and reliable feeding/launching is not yet validated. Actuator commands initially remain zero. The GUI has not been verified on your Ubuntu machine.

For commands and feedback, follow the [current package guide](src/pingpong_launcher_sim/README.md). Use `physical.launch.py` for this robot. `sim.launch.py` is the separate historical R5 demonstration that creates balls at the muzzle.

## Later sessions and updates

In every new terminal:

```bash
cd ~/ping_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

After pulling changes, rebuild before launching:

```bash
git pull
colcon build --symlink-install --packages-select pingpong_launcher_sim
source install/setup.bash
```

Current CAD/simulation scope and recorded checks: [head feeder and drain integration](src/pingpong_launcher_sim/docs/head_meter_integration.md).
