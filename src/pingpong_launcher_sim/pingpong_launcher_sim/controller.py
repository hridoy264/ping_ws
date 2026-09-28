"""Command validation and one-ball feeder sequencing using measured joint states."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger
from .control_math import JOINT_NAMES, aim_radians, wheel_radians, angle_error


class LauncherControl(Node):
    def __init__(self):
        super().__init__('launcher_control')
        self.declare_parameter('yaw_deg', 0.0)
        self.declare_parameter('elevation_deg', 0.0)
        self.declare_parameter('initial_rpm', 0.0)
        yaw, pitch = aim_radians(self.get_parameter('yaw_deg').value,
                                  self.get_parameter('elevation_deg').value)
        wheels = wheel_radians([self.get_parameter('initial_rpm').value] * 3)
        self.commands = dict(zip(JOINT_NAMES, (yaw, pitch, *wheels, 0.0)))
        self.command_publishers = {
            name: self.create_publisher(Float64, '/pingpong/' + topic + '_cmd', 10)
            for name, topic in zip(JOINT_NAMES,
                                   ('yaw', 'pitch', 'wheel_1', 'wheel_2', 'wheel_3', 'feeder'))}
        self.fire_publisher = self.create_publisher(Bool, '/pingpong/fire_request', 10)
        self.status = self.create_publisher(String, '/pingpong/status', 10)
        self.positions, self.velocities = {}, {}
        self.feedback_at = None
        self.pending = None
        self.previous_time = None
        self.create_subscription(Vector3, '/pingpong/aim', self.aim, 10)
        self.create_subscription(Vector3, '/pingpong/wheel_rpm', self.rpm, 10)
        self.create_subscription(JointState, '/pingpong/joint_states', self.feedback,
                                 qos_profile_sensor_data)
        self.create_service(Trigger, '/pingpong/fire', self.fire)
        self.create_service(Trigger, '/pingpong/stop', self.stop)
        self.create_timer(0.05, self.update)
        self.get_logger().info('Ready: aim and wheel_rpm topics; fire and stop services. '
                               'Waiting for Gazebo joint feedback.')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def report(self, message):
        self.status.publish(String(data=message))
        self.get_logger().info(message)

    def aim(self, msg):
        try:
            yaw, pitch = aim_radians(msg.x, msg.y)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return
        self.commands['yaw_joint'], self.commands['pitch_joint'] = yaw, pitch

    def rpm(self, msg):
        try:
            values = wheel_radians((msg.x, msg.y, msg.z))
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return
        for name, value in zip(JOINT_NAMES[2:5], values):
            self.commands[name] = value

    def feedback(self, msg):
        self.positions = dict(zip(msg.name, msg.position))
        self.velocities = dict(zip(msg.name, msg.velocity))
        self.feedback_at = self.now_seconds()

    def ready_to_fire(self):
        if self.feedback_at is None or not 0 <= self.now_seconds() - self.feedback_at < 1.0:
            return False, 'No recent Gazebo joint feedback.'
        if not all(name in self.positions for name in JOINT_NAMES):
            return False, 'Incomplete Gazebo joint feedback.'
        if not all(math.isfinite(self.velocities.get(name, float('nan'))) and
                   self.velocities[name] > 5.0 and self.commands[name] > 5.0
                   for name in JOINT_NAMES[2:5]):
            return False, 'Set all three wheels above 48 RPM and wait for them to spin up.'
        return True, ''

    def fire(self, request, response):
        del request
        ok, reason = self.ready_to_fire()
        if self.pending is not None:
            ok, reason = False, 'A feeder index is already in progress.'
        if not ok:
            response.success, response.message = False, reason
            return response
        target = self.positions['feeder_joint'] + math.pi / 2.0
        self.commands['feeder_joint'] = target
        self.pending = {'target': target, 'start': self.now_seconds(), 'settled_at': None}
        response.success = True
        response.message = 'Accepted: indexing feeder. Confirm the actual shot on /pingpong/shot_count.'
        self.report(response.message)
        return response

    def stop(self, request, response):
        del request
        for name in JOINT_NAMES[2:5]:
            self.commands[name] = 0.0
        self.commands['feeder_joint'] = self.positions.get('feeder_joint', 0.0)
        self.pending = None
        response.success, response.message = True, 'Wheels stopped; pending shot cancelled.'
        self.report(response.message)
        return response

    def update(self):
        now = self.now_seconds()
        if self.previous_time is not None and now < self.previous_time:
            self.pending, self.feedback_at = None, None
            self.positions, self.velocities = {}, {}
            self.commands['feeder_joint'] = 0.0
            for name in JOINT_NAMES[2:5]:
                self.commands[name] = 0.0
            self.report('Simulation reset: wheels stopped and pending shot cancelled.')
        self.previous_time = now
        for name, publisher in self.command_publishers.items():
            publisher.publish(Float64(data=float(self.commands[name])))
        if self.pending is None:
            return
        ok, reason = self.ready_to_fire()
        if not ok or now - self.pending['start'] > 5.0:
            self.pending = None
            self.report('Shot cancelled: ' + (reason or 'feeder did not reach target within 5 seconds.'))
            return
        reached = abs(angle_error(self.pending['target'], self.positions['feeder_joint'])) < 0.03
        if reached:
            if self.pending['settled_at'] is None:
                self.pending['settled_at'] = now
            elif now - self.pending['settled_at'] >= 0.2:
                self.fire_publisher.publish(Bool(data=True))
                self.pending = None
                self.report('Launch requested after feeder index; shot_count confirms ball creation.')
        else:
            self.pending['settled_at'] = None


def main(args=None):
    rclpy.init(args=args)
    node = LauncherControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
