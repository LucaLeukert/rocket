from __future__ import annotations

import math
from dataclasses import dataclass
from active_rocket_sim.core.state import ActuatorState, ForceMoment, PlantState, Vec3


@dataclass
class TVCEffector:
    thrust_n: float
    nozzle_pos_body_m: Vec3

    def compute(self, state: PlantState, actuator: ActuatorState) -> ForceMoment:
        cp = math.cos(actuator.tvc_pitch_rad)
        sp = math.sin(actuator.tvc_pitch_rad)
        cy = math.cos(actuator.tvc_yaw_rad)
        sy = math.sin(actuator.tvc_yaw_rad)
        f = Vec3(self.thrust_n * sy, -self.thrust_n * sp, self.thrust_n * cp * cy)
        arm = self.nozzle_pos_body_m - state.mass.com_body_m
        m = arm.cross(f)
        return ForceMoment(force_b_n=f, moment_b_nm=m, source="tvc")
