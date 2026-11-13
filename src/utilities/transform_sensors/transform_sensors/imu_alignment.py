"""IMU leveling from the stored static calibration, as one rotation for gyro and accel.

The calibration file stores ``mean_static_acc - [0, 0, 9.81]`` in the frame after
the fixed 15.1 degree tilt. When that frame is not actually level, the horizontal
part is gravity, not sensor bias. Subtracting it as a constant is exact only in
the calibration posture; any later tilt d produces a false horizontal
acceleration of about |tilt residual| * g * d. A rotation is exact in every posture.
"""

import math

GRAVITY = 9.81


def _rotation_to_z(u):
    x, y, z = u
    s = math.hypot(x, y)
    if s < 1e-12:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)) if z > 0 else None
    # Rodrigues for the minimal rotation u -> e_z; no yaw is introduced.
    kx, ky = y / s, -x / s
    angle = math.atan2(s, z)
    c, si, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return ((c + kx * kx * t, kx * ky * t, ky * si),
            (kx * ky * t, c + ky * ky * t, -kx * si),
            (-ky * si, kx * si, c))


def _apply(r, v):
    return tuple(r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2] for i in range(3))


class GravityAlignment:
    """Rotate the tilted IMU frame so measured static gravity is +z."""

    def __init__(self, calibration, max_tilt_deg=30.0):
        """Derive the rotation and residual biases from the eight-value calibration."""
        mean = (calibration['acc_bias_x'], calibration['acc_bias_y'],
                calibration['acc_bias_z'] + GRAVITY)
        norm = math.sqrt(sum(v * v for v in mean))
        if not math.isfinite(norm) or not 0.8 * GRAVITY <= norm <= 1.2 * GRAVITY:
            raise ValueError('Static calibration acceleration is not gravity-like')
        up = tuple(v / norm for v in mean)
        self.tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))
        if self.tilt_deg > max_tilt_deg:
            raise ValueError(
                'Calibration tilt %.1f deg exceeds %.1f deg' % (self.tilt_deg, max_tilt_deg))
        self.rotation = _rotation_to_z(up)
        self.gyro_bias = (
            calibration['ang_bias_x'], calibration['ang_bias_y'], calibration['ang_bias_z'])
        self.acc_z_bias = norm - GRAVITY
        spin = (-calibration['ang_z2x_proj'], -calibration['ang_z2y_proj'], 1.0)
        spin_norm = math.sqrt(sum(v * v for v in spin))
        leveled_spin = _apply(self.rotation, tuple(v / spin_norm for v in spin))
        # Independent check from the yaw spin: both must describe the same vertical.
        self.spin_residual_deg = math.degrees(math.acos(max(-1.0, min(1.0, leveled_spin[2]))))

    def gyro(self, x, y, z):
        """Remove static gyro bias, then rotate into the leveled frame."""
        b = self.gyro_bias
        return _apply(self.rotation, (x - b[0], y - b[1], z - b[2]))

    def accel(self, x, y, z):
        """Rotate into the leveled frame; only the gravity magnitude error remains as bias."""
        ax, ay, az = _apply(self.rotation, (x, y, z))
        return ax, ay, az - self.acc_z_bias
