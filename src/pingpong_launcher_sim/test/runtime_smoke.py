#!/usr/bin/env python3
"""Exercise the actual headless ROS/Gazebo stack on Ubuntu 22.04.

After colcon build and sourcing install/setup.bash:
  python3 src/pingpong_launcher_sim/test/runtime_smoke.py

Starts and stops its own simulation by default. Use --attach to exercise an
already-running instance. The test moves all six joints and launches five balls.
It checks measured simulation feedback; a successful publish is not a pass.
"""

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


JOINTS = {"yaw_joint", "pitch_joint", "feeder_joint",
          "wheel_1_joint", "wheel_2_joint", "wheel_3_joint"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attach", action="store_true", help="Do not start or stop Gazebo")
    parser.add_argument("--shots", type=int, default=5, help="Shots to test; five crosses a full feeder revolution")
    parser.add_argument("--timeout", type=float, default=60.0,
                        help="Wall-clock seconds allowed per readiness/action check")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.shots < 1:
        parser.error("--shots must be positive")

    # Keep imports here so --help is useful outside a ROS installation.
    try:
        import rclpy
        from geometry_msgs.msg import Vector3
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String, UInt32
        from std_srvs.srv import Trigger
        from tf2_msgs.msg import TFMessage
    except ImportError as error:
        print(f"ROS 2 Humble Python environment is unavailable: {error}\n"
              "Source /opt/ros/humble/setup.bash and this workspace's install/setup.bash.",
              file=sys.stderr)
        return 2

    class Probe(Node):
        def __init__(self):
            super().__init__("pingpong_runtime_smoke")
            self.positions, self.velocities = {}, {}
            self.feedback_time = 0.0
            self.clock = None
            self.shots = None
            self.status, self.ball_status = "", ""
            self.ball_positions = {}
            self.create_subscription(JointState, "/pingpong/joint_states", self.joints,
                                     qos_profile_sensor_data)
            self.create_subscription(Clock, "/clock", self.clock_received, qos_profile_sensor_data)
            self.create_subscription(UInt32, "/pingpong/shot_count", self.shot_received, 10)
            self.create_subscription(String, "/pingpong/status", self.status_received, 10)
            self.create_subscription(String, "/pingpong/ball_status", self.ball_status_received, 10)
            self.create_subscription(TFMessage, "/pingpong/dynamic_poses", self.poses_received,
                                     qos_profile_sensor_data)
            self.aim = self.create_publisher(Vector3, "/pingpong/aim", 10)
            self.wheels = self.create_publisher(Vector3, "/pingpong/wheel_rpm", 10)
            self.fire = self.create_client(Trigger, "/pingpong/fire")
            self.stop = self.create_client(Trigger, "/pingpong/stop")

        def joints(self, message):
            self.positions.update(zip(message.name, message.position))
            self.velocities.update(zip(message.name, message.velocity))
            self.feedback_time = time.monotonic()

        def clock_received(self, message):
            self.clock = message.clock.sec + message.clock.nanosec * 1e-9

        def shot_received(self, message):
            self.shots = message.data

        def status_received(self, message):
            self.status = message.data

        def ball_status_received(self, message):
            self.ball_status = message.data

        def poses_received(self, message):
            for transform in message.transforms:
                name = transform.child_frame_id
                if name.startswith('pingpong_ball_') and '::' not in name:
                    p = transform.transform.translation
                    self.ball_positions[name] = (p.x, p.y, p.z)

        def diagnostics(self):
            return (f"positions={self.positions}\nvelocities={self.velocities}\n"
                    f"shot_count={self.shots}, clock={self.clock}\n"
                    f"controller={self.status}\nlauncher={self.ball_status}")

    launch = None
    log = None
    log_path = None
    probe = None
    success = False
    rclpy.init()

    try:
        if not args.attach:
            log_directory = Path(tempfile.mkdtemp(prefix="pingpong-smoke-"))
            log_path = log_directory / "simulation.log"
            log = log_path.open("w")
            env = dict(os.environ)
            env["ROS_LOG_DIR"] = str(log_directory / "ros")
            launch = subprocess.Popen(
                ["ros2", "launch", "pingpong_launcher_sim", "sim.launch.py", "gui:=false"],
                stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
            print(f"Starting headless Gazebo; logs: {log_path}", flush=True)
        probe = Probe()

        def wait_until(predicate, description, publish=None, timeout=None):
            deadline = time.monotonic() + (args.timeout if timeout is None else timeout)
            next_publish = 0.0
            while time.monotonic() < deadline:
                if launch is not None and launch.poll() is not None:
                    raise RuntimeError(f"Simulation exited with code {launch.returncode}")
                now = time.monotonic()
                if publish is not None and now >= next_publish:
                    publish()
                    next_publish = now + 0.2
                rclpy.spin_once(probe, timeout_sec=0.05)
                if predicate():
                    print(f"PASS {description}", flush=True)
                    return
            raise RuntimeError(f"Timed out: {description}\n{probe.diagnostics()}")

        def fresh():
            return time.monotonic() - probe.feedback_time < 2.0

        def all_feedback():
            return (fresh() and JOINTS.issubset(probe.positions) and JOINTS.issubset(probe.velocities)
                    and all(math.isfinite(probe.positions[name]) and math.isfinite(probe.velocities[name])
                            for name in JOINTS))

        def call(client, name):
            future = client.call_async(Trigger.Request())
            wait_until(future.done, name + " service responded", timeout=10.0)
            response = future.result()
            if response is None:
                raise RuntimeError(f"{name} service failed: {future.exception()}")
            return response

        def fire():
            return call(probe.fire, 'fire')

        wait_until(lambda: all_feedback() and probe.clock is not None and probe.shots is not None
                   and probe.fire.service_is_ready() and probe.stop.service_is_ready(),
                   "six articulated joints, ROS clock, and fire service are available")
        initial_clock = probe.clock
        wait_until(lambda: probe.clock > initial_clock + 0.05, "simulation time advances")
        wait_until(lambda: probe.aim.get_subscription_count() > 0 and probe.wheels.get_subscription_count() > 0,
                   "high-level command subscribers are connected")

        stopped = Vector3(x=0.0, y=0.0, z=0.0)
        wait_until(lambda: fresh() and all(abs(probe.velocities[f"wheel_{i}_joint"]) < 0.5
                                          for i in (1, 2, 3)),
                   "all wheels are stopped", publish=lambda: probe.wheels.publish(stopped))
        # Let the controller consume the same feedback sample before the service.
        settle_deadline = time.monotonic() + 0.3
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)
        response = fire()
        if response.success:
            raise RuntimeError("Fire was accepted with stopped wheels")
        print(f"PASS fire with stopped wheels was rejected: {response.message}", flush=True)

        aim = Vector3(x=10.0, y=10.0, z=0.0)
        angle = math.radians(10.0)
        wait_until(lambda: fresh() and abs(probe.positions["yaw_joint"] - angle) < 0.035
                   and abs(probe.positions["pitch_joint"] + angle) < 0.035,
                   "measured aim reaches +10 deg yaw and +10 deg elevation",
                   publish=lambda: probe.aim.publish(aim))

        rpm = (900.0, 1050.0, 1200.0)
        wheels = Vector3(x=rpm[0], y=rpm[1], z=rpm[2])
        rates = tuple(value * 2 * math.pi / 60 for value in rpm)
        wait_until(lambda: fresh() and all(
            abs(probe.velocities[f"wheel_{i}_joint"] - rate) < max(5.0, rate * 0.1)
            for i, rate in enumerate(rates, 1)),
            "all three measured wheel velocities match their individual RPM commands",
            publish=lambda: probe.wheels.publish(wheels))

        previous_shots = probe.shots
        previous_feeder = probe.positions["feeder_joint"]
        response = fire()
        if not response.success:
            raise RuntimeError(f"Fire with spinning wheels was rejected: {response.message}")
        wait_until(lambda: probe.shots == previous_shots + 1,
                   "Gazebo confirms one launched ball (not merely a queued fire request)")
        wait_until(lambda: fresh() and abs(probe.positions["feeder_joint"] - previous_feeder - math.pi / 2) < 0.1,
                   "feeder physically advances one 90 deg pocket")
        wait_until(lambda: bool(probe.ball_positions), "spawned ball has a world pose")
        ball_name = sorted(probe.ball_positions)[-1]
        initial_position = probe.ball_positions[ball_name]
        wait_until(lambda: probe.ball_positions[ball_name][0] > initial_position[0] + 0.10,
                   "launched ball keeps moving forward by at least 10 cm")
        for shot in range(2, args.shots + 1):
            feeder_before = probe.positions['feeder_joint']
            response = fire()
            if not response.success:
                raise RuntimeError(f'Repeated shot {shot} rejected: {response.message}')
            wait_until(lambda: probe.shots == previous_shots + shot,
                       f'repeated shot {shot} is confirmed')
            wait_until(lambda: fresh() and abs(probe.positions['feeder_joint'] - feeder_before
                                               - math.pi / 2) < 0.1,
                       f'feeder index {shot} reaches its measured target')
        # A stop must also cancel a feeder sequence which has been accepted but
        # has not yet reached the launch boundary.
        response = fire()
        if not response.success:
            raise RuntimeError('Could not queue shot for stop/cancellation test')
        response = call(probe.stop, 'stop')
        if not response.success:
            raise RuntimeError('Stop service failed: ' + response.message)
        wait_until(lambda: fresh() and all(abs(probe.velocities[f"wheel_{i}_joint"]) < 0.5
                                          for i in (1, 2, 3)),
                   "stop service stops all measured wheel velocities")
        cancelled_at = probe.clock
        wait_until(lambda: probe.clock > cancelled_at + 1.25, 'cancelled shot observation period elapsed')
        if probe.shots != previous_shots + args.shots:
            raise RuntimeError("Stop did not cancel the pending shot, or a request launched multiple balls")
        print('PASS stop cancels a pending feeder/launch sequence', flush=True)
        success = True
        print("PASS ROS 2 Humble / Gazebo Fortress runtime smoke test", flush=True)
    except (RuntimeError, OSError, KeyboardInterrupt) as error:
        print(f"FAIL {error}", file=sys.stderr, flush=True)
    finally:
        if probe is not None:
            # Also stop wheels when an assertion fails in an attached simulation.
            for _ in range(3):
                probe.wheels.publish(Vector3(x=0.0, y=0.0, z=0.0))
                rclpy.spin_once(probe, timeout_sec=0.05)
            probe.destroy_node()
        if launch is not None and launch.poll() is None:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait(timeout=5)
        if log is not None:
            log.close()
        if not success and log_path is not None:
            print(f"\nLast simulation log lines ({log_path}):", file=sys.stderr)
            print("\n".join(log_path.read_text(errors="replace").splitlines()[-70:]), file=sys.stderr)
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
