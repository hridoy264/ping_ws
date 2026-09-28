"""Public command units, rejected inputs, and cyclic feeder-angle boundaries."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pingpong_launcher_sim.control_math import aim_radians, angle_error, wheel_radians  # noqa: E402


class ControlMath(unittest.TestCase):
    def test_aim_sign_matches_positive_elevation_about_y(self):
        yaw, pitch = aim_radians(10.0, 10.0)
        self.assertAlmostEqual(yaw, math.pi / 18)
        self.assertAlmostEqual(pitch, -math.pi / 18)
        # A negative rotation around +Y raises the muzzle's +X launch vector.
        self.assertGreater(-math.sin(pitch), 0)

    def test_aim_accepts_mechanical_endpoints(self):
        self.assertEqual(aim_radians(0.0, 0.0), (0.0, 0.0))
        for yaw in (-25.0, 25.0):
            for elevation in (-10.0, 20.0):
                self.assertTrue(all(math.isfinite(value) for value in aim_radians(yaw, elevation)))

    def test_aim_rejects_even_just_outside_mechanical_limits(self):
        for yaw, elevation in ((math.nextafter(-25, -math.inf), 0),
                               (math.nextafter(25, math.inf), 0),
                               (0, math.nextafter(-10, -math.inf)),
                               (0, math.nextafter(20, math.inf))):
            with self.subTest(yaw=yaw, elevation=elevation), self.assertRaises(ValueError):
                aim_radians(yaw, elevation)

    def test_nonfinite_aim_never_reaches_gazebo(self):
        for invalid in (math.nan, math.inf, -math.inf):
            with self.subTest(value=invalid):
                with self.assertRaises(ValueError):
                    aim_radians(invalid, 0)
                with self.assertRaises(ValueError):
                    aim_radians(0, invalid)

    def test_individual_rpm_channels_and_bounds(self):
        expected = (0.0, 2.0 * math.pi, 200.0 * math.pi)
        for actual, target in zip(wheel_radians((0.0, 60.0, 6000.0)), expected):
            self.assertAlmostEqual(actual, target)

    def test_invalid_wheel_commands_are_rejected_as_a_group(self):
        for speeds in ((), (1.0,), (1.0, 2.0), (1.0, 2.0, 3.0, 4.0)):
            with self.subTest(speeds=speeds), self.assertRaises(ValueError):
                wheel_radians(speeds)
        for invalid in (-0.01, math.nextafter(6000, math.inf), math.nan, math.inf, -math.inf):
            for channel in range(3):
                speeds = [100.0, 100.0, 100.0]
                speeds[channel] = invalid
                with self.subTest(channel=channel, speeds=speeds), self.assertRaises(ValueError):
                    wheel_radians(speeds)

    def test_feeder_wraps_across_a_complete_revolution(self):
        self.assertAlmostEqual(angle_error(2 * math.pi + 0.01, 0), 0.01)
        self.assertAlmostEqual(angle_error(0.01, 2 * math.pi - 0.01), 0.02)
        self.assertAlmostEqual(angle_error(2 * math.pi - 0.01, 0.01), -0.02)
        self.assertAlmostEqual(angle_error(5 * math.pi / 2, math.pi / 2), 0.0)


if __name__ == "__main__":
    unittest.main()
