from __future__ import annotations

import math
from .state import Quaternion, Vec3


def quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    return Quaternion(
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    )


def quat_from_omega_dt(omega: Vec3, dt: float) -> Quaternion:
    half = 0.5 * dt
    return Quaternion(1.0, omega.x * half, omega.y * half, omega.z * half).normalized()


def rotate_body_to_inertial(q_bi: Quaternion, v_b: Vec3) -> Vec3:
    qv = Quaternion(0.0, v_b.x, v_b.y, v_b.z)
    qc = Quaternion(q_bi.w, -q_bi.x, -q_bi.y, -q_bi.z)
    out = quat_multiply(quat_multiply(q_bi, qv), qc)
    return Vec3(out.x, out.y, out.z)


def euler_from_quaternion(q: Quaternion) -> tuple[float, float, float]:
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw
