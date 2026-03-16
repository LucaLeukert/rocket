from __future__ import annotations

import random
from dataclasses import dataclass
from active_rocket_sim.core.state import MeasurementPacket, PlantState


@dataclass
class Barometer:
    noise_std_m: float = 0.5

    def sample(self, state: PlantState) -> MeasurementPacket:
        return MeasurementPacket(
            sensor="baro",
            t=state.t,
            values={"altitude_m": state.position_i_m.z + random.gauss(0.0, self.noise_std_m)},
        )
