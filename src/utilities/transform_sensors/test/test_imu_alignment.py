"""Gravity leveling versus the legacy constant horizontal bias; no node or hardware."""

import math
import unittest

from transform_sensors.imu_alignment import GravityAlignment

# Values from ~/Desktop/imu_calib_data.yaml used on the robot (audit 2026-10-31).
ROBOT = dict(acc_bias_x=-1.62771, acc_bias_y=2.33169, acc_bias_z=-0.369387,
             ang_bias_x=-0.00012074, ang_bias_y=-0.000620588, ang_bias_z=-0.00262201,
             ang_z2x_proj=0.19147, ang_z2y_proj=-0.254826)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return ((1, 0, 0), (0, c, -s), (0, s, c))


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0, s), (0, 1, 0), (-s, 0, c))


def mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def apply(r, v):
    return tuple(sum(r[i][k] * v[k] for k in range(3)) for i in range(3))


def transpose(r):
    return tuple(tuple(r[j][i] for j in range(3)) for i in range(3))


class GravityAlignmentTests(unittest.TestCase):
    def test_negative_yaw_maps_accel_and_bias_corrected_gyro_to_same_axes(self):
        calibration = dict(ROBOT, acc_bias_x=0, acc_bias_y=0, acc_bias_z=0)
        a = GravityAlignment(calibration, yaw_correction_deg=-90)
        acc = a.accel(1, 0, 9.81)
        gyro = a.gyro(1 + ROBOT['ang_bias_x'], ROBOT['ang_bias_y'], ROBOT['ang_bias_z'])
        for actual, expected in zip(acc, (0, -1, 9.81)):
            self.assertAlmostEqual(actual, expected, places=10)
        for actual, expected in zip(gyro, (0, -1, 0)):
            self.assertAlmostEqual(actual, expected, places=10)

    def test_yaw_after_leveling_preserves_gravity_and_vector_lengths(self):
        base = GravityAlignment(ROBOT)
        corrected = GravityAlignment(ROBOT, yaw_correction_deg=-57)
        mean = (ROBOT['acc_bias_x'], ROBOT['acc_bias_y'], ROBOT['acc_bias_z'] + 9.81)
        for actual, expected in zip(corrected.accel(*mean), (0, 0, 9.81)):
            self.assertAlmostEqual(actual, expected, places=9)
        source = (1.2, -2.3, 8.7)
        x, y, z = base.accel(*source)
        c, s = math.cos(math.radians(-57)), math.sin(math.radians(-57))
        expected = (c * x - s * y, s * x + c * y, z)
        for actual, want in zip(corrected.accel(*source), expected):
            self.assertAlmostEqual(actual, want, places=10)
        self.assertAlmostEqual(math.dist(corrected.gyro(*source), (0, 0, 0)),
                               math.dist(base.gyro(*source), (0, 0, 0)), places=10)

    def test_rejects_invalid_yaw(self):
        for value in (True, float('nan'), float('inf'), -180.01, 180.01, '57'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                GravityAlignment(ROBOT, yaw_correction_deg=value)

    def test_robot_calibration_levels_static_gravity_and_spin_axis(self):
        a = GravityAlignment(ROBOT)
        mean = (ROBOT['acc_bias_x'], ROBOT['acc_bias_y'], ROBOT['acc_bias_z'] + 9.81)
        ax, ay, az = a.accel(*mean)
        self.assertAlmostEqual(ax, 0, places=9)
        self.assertAlmostEqual(ay, 0, places=9)
        self.assertAlmostEqual(az, 9.81, places=9)
        self.assertAlmostEqual(a.tilt_deg, 16.763, places=2)
        # Yaw-spin axis is an independent measurement of the same vertical.
        self.assertLess(a.spin_residual_deg, 1.5)

    def test_tilted_posture_has_no_false_horizontal_acceleration(self):
        # Sensor frame F is tilted T from level body; robot later pitches by d.
        mount = mul(rot_x(math.radians(13.9)), rot_y(math.radians(-9.8)))
        g_up = (0.0, 0.0, 9.81)
        mean = apply(transpose(mount), g_up)
        calibration = dict(acc_bias_x=mean[0], acc_bias_y=mean[1], acc_bias_z=mean[2] - 9.81,
                           ang_bias_x=0, ang_bias_y=0, ang_bias_z=0, ang_z2x_proj=0, ang_z2y_proj=0)
        a = GravityAlignment(calibration)
        d = math.radians(3.0)
        posture = rot_y(d)
        measured = apply(transpose(mount), apply(transpose(posture), g_up))
        expected = apply(transpose(posture), g_up)
        got = a.accel(*measured)
        legacy = tuple(m - b for m, b in zip(measured, (mean[0], mean[1], mean[2] - 9.81)))
        new_error = math.dist(got, expected)
        legacy_error = math.dist(legacy, expected)
        # Residual comes only from the mount yaw, which gravity cannot observe.
        self.assertLess(new_error, 0.02)
        self.assertGreater(legacy_error, 0.08)
        self.assertLess(new_error * 5, legacy_error)

    def test_rejects_non_gravity_calibration(self):
        bad = dict(ROBOT, acc_bias_z=-9.81)
        with self.assertRaises(ValueError):
            GravityAlignment(bad)


if __name__ == '__main__':
    unittest.main()
