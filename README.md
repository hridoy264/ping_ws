# PingPong R5 simulation workspace

Target: **Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress**.

Install ROS 2 Humble first, extract this source workspace into your home directory,
then run:

```bash
cd ~/ros2_ws
bash src/pingpong_launcher_sim/scripts/install_ubuntu_dependencies.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pingpong_launcher_sim
source install/setup.bash
ros2 launch pingpong_launcher_sim sim.launch.py
```

See [the package guide](src/pingpong_launcher_sim/README.md) for aim, RPM, fire,
and stop commands. [Validation results](src/pingpong_launcher_sim/docs/validation.md)
cover the real Ubuntu build and headless simulator tests.

Ball release is an approximation based on measured simulated wheel speeds.
Spin, wheel/ball compression, and physical feeding through the hose are not yet
modeled. No Fusion installation or external CAD files are needed to run.

Use the source archive when transferring to another machine. The `build`,
`install`, and `log` directories in the development checkout belong to its
Docker test environment and must not be copied to Ubuntu.
