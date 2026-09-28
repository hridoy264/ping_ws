# Validation record — 28 September 2026

The package was built and run in an **Ubuntu 22.04 ARM64 container**, using
ROS 2 Humble, Python 3.10.12, Gazebo Fortress 6.18.0, SDFormat 12.9.0 and
the DART physics backend. The container ran on Docker Desktop on the development
Mac. These are real simulator tests, not just XML or source inspection.

## Passed

- `rosdep check --from-paths src --ignore-src --rosdistro humble`: all declared
  dependencies resolve and are installed.
- `colcon build --symlink-install --packages-select pingpong_launcher_sim`:
  the C++ ball system and Python control node build and install.
- `colcon test`: 14 Python assertions/tests across two CTest suites, no failures.
  `colcon test-result` reports 16 total results because it includes both suites.
- `ign sdf -k` validates the launcher model and the training world (with
  `SDF_PATH` pointing to the package's model directory).
- The headless runtime test observed advancing simulation time and measured
  positions/velocities for all six moving joints through the ROS bridge.
- A fire request with stopped wheels was rejected.
- The measured yaw/pitch reached the commanded +10° heading/+10° elevation
  within the test's 2° tolerance, with the correct pitch sign.
- Three different commanded wheel speeds (900, 1050, 1200 RPM) reached the
  individual simulated joints within the test's 10% velocity tolerance.
- Five consecutive requests each produced exactly one confirmed launch and
  one measured 90° feeder increment, including travel beyond 360°.
- The launched ball's world position advanced by more than 10 cm after launch,
  confirming sustained physics motion rather than a successful spawn alone.
- Stop commanded all measured wheel rates below 0.5 rad/s and cancelled a
  pending feeder/launch sequence without an extra ball.

The recorded runtime output is in [runtime_smoke.log](runtime_smoke.log).
Build and test summaries are in [build.log](build.log) and
[unit_tests.log](unit_tests.log). The test scripts are shipped so these checks
can be repeated on the target Ubuntu machine.

## Scope and remaining checks

The Ubuntu GUI and GPU renderer were not exercised in the headless container.
The x86-64 build was not independently run; this is portable source targeting
the same Humble/Fortress libraries on Ubuntu 22.04, not an ARM-only binary release.
The source archive excludes machine-specific `build`, `install`, and `log`
directories and Python caches.

This validates software integration and the declared simplified model. It does
not validate hardware masses/inertias, actual motor torque limits, spin or
Magnus forces, wheel-to-ball contact/compression, flexible hose behavior,
hopper feeding, jam detection, bounce accuracy, or real launch performance.
The rotating pinion visuals are cosmetic, and geared actuator dynamics are
represented by ideal joint commands. See [geometry.md](geometry.md) and the
package README before using trajectory or force estimates for design decisions.
