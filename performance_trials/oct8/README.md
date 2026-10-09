# R10 performance isolation — 8 October 2026

These are diagnostic copies. The canonical 1 ms world and its failed runtime
reports remain unchanged. The exact source model and collision meshes are
snapshotted under `models/`; SHA-256 values are recorded in the manifest.

| Case | Balls | Robot collisions | Purpose |
|---|---:|---:|---|
| full_1 | 1 | 377 | Complete robot with one ball |
| full_10 | 10 | 377 | Complete robot with a small batch |
| full_100 | 100 | 377 | Complete robot and full load |
| static_robot_100 | 100 | 377 | Same geometry, frozen joints and no actuator controllers |
| hopper_only_100 | 100 | 17 | Hopper, neck and feeder floor only |
| floor_only_100 | 100 | 0 | Ball and environment contact baseline |
| full_100_pgs | 100 | 377 | Complete robot, DART PGS constraint solver |
| full_100_bullet_pgs | 100 | 377 | Complete robot, DART with Bullet collision detection and PGS |

All copies use a 1 ms step. Full copies keep the real physical joints and
controllers with zero wheel and feeder commands. Stripped/static copies cannot
establish robot performance, feeding, or capacity. The same named physical balls
are retained; no teleportation, spawning service, expiration or artificial launch
is added. The passive inventory monitor's expected count matches each input.

The two solver variants are optional (`--cases full_100_pgs
full_100_bullet_pgs`). Fortress documents `pgs` and `dantzig` constraint solvers,
and `ode`, `bullet`, `fcl`, `dart` collision detectors within the DART engine.
The Bullet collision-detector variant still uses DART dynamics; it does not
switch to the separate experimental Bullet physics plugin. Actual installed
support and numerical behaviour remain subject to native execution and log
inspection. Source: [Fortress physics-engine configuration](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/tutorials/physics.md#engine-configuration).

Run inside the existing container after the workspace has been built:

```bash
cd /ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="/ws/install/pingpong_launcher_sim/lib:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}"
python3 src/pingpong_launcher_sim/tools/run_performance_trials.py \
  --directory performance_trials/oct8 --duration 2 --timeout 60
```

The runner prepends the frozen model resource directory, launches cases
sequentially with unique transport partitions, saves per-case reports/logs, and
updates `benchmark_summary.json` after every case. Inventory snapshots now carry
wall time relative to launch so startup delay can be separated from simulation
cost. `observed_realtime_factor` covers the interval between the first and last
received complete snapshots. It includes transport/scheduling delay and is not a
pure physics-profiler measurement.

## Existing measurements reviewed

- Canonical 1 ms optimized world: 150 wall seconds, 1.002 simulated seconds,
  100 finite persistent IDs; failed timeout.
- Previous 5 ms stopped-wheel diagnostic: 300 wall seconds, 4.265 simulated
  seconds; failed timeout. Sampled maximum ball-pair penetration was 2.85 mm,
  above the 1 mm criterion. Five balls still exceeded the rim; highest whole-ball
  top was 7.33 mm above the rim. It cannot establish 100-ball capacity or justify
  using the larger step for launch simulation.
- Bullet plugin discovery from the prior turn is in
  `../bullet_plugin_info.txt`; its existence alone does not establish compatibility
  with the complete Gazebo model. No alternative engine run has passed.

Initial October 8 availability checks found Docker's daemon and desktop status
requests timed out. The final backend log was an engine transition to stopping
at 01:30:53 Dhaka. No new native benchmark result should be inferred from prepared
world files. See saved Docker preflight JSON files and any later benchmark reports
for the subsequent runtime state.

At the afternoon continuation, `docker desktop start` reported already running.
The supported `docker desktop restart --timeout 45` then failed to stop the
stale backend processes (PIDs 67053, 67054, 67055 and build process 67120) with a
context deadline exceeded error. `docker_recovery.json` retains the exact CLI
output. No native ablation run has been completed at this checkpoint. All eight
diagnostics passed source-level checks of inventory count, collision count,
resource existence, step size and saved-input hashes; that is not a runtime pass.
