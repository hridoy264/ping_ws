"""Unit conversions and input validation; no ROS runtime required."""
import math

JOINT_NAMES = ('yaw_joint', 'pitch_joint', 'wheel_1_joint', 'wheel_2_joint',
               'wheel_3_joint', 'feeder_joint')


def aim_radians(yaw_deg, elevation_deg):
    if not all(math.isfinite(v) for v in (yaw_deg, elevation_deg)):
        raise ValueError('Aim must contain finite angles.')
    if not -25.0 <= yaw_deg <= 25.0:
        raise ValueError('Yaw must be between -25 and +25 degrees.')
    if not -10.0 <= elevation_deg <= 20.0:
        raise ValueError('Elevation must be between -10 and +20 degrees.')
    return math.radians(yaw_deg), -math.radians(elevation_deg)


def wheel_radians(rpms):
    if len(rpms) != 3 or not all(math.isfinite(v) and 0 <= v <= 6000 for v in rpms):
        raise ValueError('Provide three finite wheel speeds between 0 and 6000 RPM.')
    return tuple(v * math.tau / 60.0 for v in rpms)


def angle_error(target, measured):
    return math.atan2(math.sin(target - measured), math.cos(target - measured))
