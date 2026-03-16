from __future__ import annotations

from dataclasses import dataclass
from .state import PlantState


@dataclass
class EventFlags:
    ignition: bool = False
    rail_exit: bool = False
    burnout: bool = False
    apogee: bool = False


class EventDetector:
    def __init__(self, rail_exit_altitude_m: float = 1.0):
        self.rail_exit_altitude_m = rail_exit_altitude_m
        self._prev_vz = 0.0

    def update(self, state: PlantState, burn_remaining_s: float) -> EventFlags:
        flags = EventFlags()
        flags.ignition = state.t <= 1e-9
        flags.rail_exit = state.position_i_m.z > self.rail_exit_altitude_m
        flags.burnout = burn_remaining_s <= 0.0
        flags.apogee = self._prev_vz > 0.0 and state.velocity_i_ms.z <= 0.0
        self._prev_vz = state.velocity_i_ms.z
        state.events.update(
            {
                "ignition": flags.ignition,
                "rail_exit": flags.rail_exit,
                "burnout": flags.burnout,
                "apogee": flags.apogee,
            }
        )
        state.on_rail = not flags.rail_exit
        return flags
