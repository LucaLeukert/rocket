from __future__ import annotations

from active_rocket_sim.core.state import EstimatorState, PlantState


class ModeManager:
    def __init__(self):
        self.mode = "prelaunch"

    def update(self, truth: PlantState, est: EstimatorState) -> str:
        if truth.events.get("apogee"):
            self.mode = "coast"
        elif truth.events.get("burnout"):
            self.mode = "burnout_transition"
        elif truth.events.get("rail_exit"):
            self.mode = "ascent_stabilize"
        elif truth.events.get("ignition"):
            self.mode = "rail_ascent"
        return self.mode
