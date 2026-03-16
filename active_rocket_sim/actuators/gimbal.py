from __future__ import annotations

from dataclasses import dataclass
from active_rocket_sim.core.state import ActuatorState, CommandVector


@dataclass
class GimbalActuator:
    max_rate_rads: float = 7.0
    deadband_rad: float = 0.002
    max_angle_rad: float = 0.25

    def propagate(self, cmd: CommandVector, state: ActuatorState, dt: float) -> ActuatorState:
        def step_axis(target: float, x: float) -> tuple[float, float]:
            err = target - x
            if abs(err) < self.deadband_rad:
                return x, 0.0
            rate = max(-self.max_rate_rads, min(self.max_rate_rads, err / max(dt, 1e-6)))
            nx = x + rate * dt
            nx = max(-self.max_angle_rad, min(self.max_angle_rad, nx))
            return nx, rate

        p, pd = step_axis(cmd.tvc_pitch_rad, state.tvc_pitch_rad)
        y, yd = step_axis(cmd.tvc_yaw_rad, state.tvc_yaw_rad)
        return ActuatorState(tvc_pitch_rad=p, tvc_yaw_rad=y, tvc_pitch_rate_rads=pd, tvc_yaw_rate_rads=yd)
