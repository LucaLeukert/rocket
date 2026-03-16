from __future__ import annotations

import random
from dataclasses import dataclass
from active_rocket_sim.core.state import MeasurementPacket, PlantState


@dataclass
class GNSS:
    pos_noise_std_m: float = 1.5
    vel_noise_std_ms: float = 0.2

    def sample(self, state: PlantState) -> MeasurementPacket:
        return MeasurementPacket(
            sensor="gnss",
            t=state.t,
            values={
                "x_m": state.position_i_m.x + random.gauss(0.0, self.pos_noise_std_m),
                "y_m": state.position_i_m.y + random.gauss(0.0, self.pos_noise_std_m),
                "z_m": state.position_i_m.z + random.gauss(0.0, self.pos_noise_std_m),
                "vz_ms": state.velocity_i_ms.z + random.gauss(0.0, self.vel_noise_std_ms),
            },
        )
