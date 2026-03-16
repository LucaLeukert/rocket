from __future__ import annotations

import random
from dataclasses import dataclass
from active_rocket_sim.core.state import MeasurementPacket, PlantState


@dataclass
class IMUSensor:
    gyro_noise_std: float = 0.002
    accel_noise_std: float = 0.05

    def sample(self, state: PlantState) -> MeasurementPacket:
        return MeasurementPacket(
            sensor="imu",
            t=state.t,
            values={
                "gx": state.omega_b_rads.x + random.gauss(0.0, self.gyro_noise_std),
                "gy": state.omega_b_rads.y + random.gauss(0.0, self.gyro_noise_std),
                "gz": state.omega_b_rads.z + random.gauss(0.0, self.gyro_noise_std),
                "az": -state.environment.gravity_ms2 + random.gauss(0.0, self.accel_noise_std),
            },
        )
