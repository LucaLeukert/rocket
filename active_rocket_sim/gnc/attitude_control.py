from __future__ import annotations

from dataclasses import dataclass

from active_rocket_sim.core.frames import euler_from_quaternion
from active_rocket_sim.core.state import CommandVector, EstimatorState


@dataclass
class CascadedController:
    kp_att: float = 1.5
    kd_rate: float = 0.15
    max_gimbal_rad: float = 0.2

    def update(self, est: EstimatorState, mode: str, dt: float) -> CommandVector:
        _roll, pitch, yaw = euler_from_quaternion(est.attitude_bi)
        if mode in {"prelaunch", "rail_ascent"}:
            return CommandVector()

        pitch_cmd = -(self.kp_att * pitch + self.kd_rate * est.omega_b_rads.y)
        yaw_cmd = -(self.kp_att * yaw + self.kd_rate * est.omega_b_rads.x)
        pitch_cmd = max(-self.max_gimbal_rad, min(self.max_gimbal_rad, pitch_cmd))
        yaw_cmd = max(-self.max_gimbal_rad, min(self.max_gimbal_rad, yaw_cmd))
        return CommandVector(tvc_pitch_rad=pitch_cmd, tvc_yaw_rad=yaw_cmd)
