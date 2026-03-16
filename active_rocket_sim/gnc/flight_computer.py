from __future__ import annotations

from dataclasses import dataclass

from active_rocket_sim.core.scheduler import RateGate
from active_rocket_sim.core.state import CommandVector, EstimatorState, MeasurementPacket
from active_rocket_sim.estimation.attitude_cf import AttitudeComplementaryFilter
from active_rocket_sim.gnc.attitude_control import CascadedController
from active_rocket_sim.gnc.modes import ModeManager


@dataclass
class FlightComputer:
    imu_rate_hz: float = 500.0
    ctrl_rate_hz: float = 200.0

    def __post_init__(self) -> None:
        self.estimator = AttitudeComplementaryFilter()
        self.controller = CascadedController()
        self.modes = ModeManager()
        self.imu_gate = RateGate(self.imu_rate_hz)
        self.ctrl_gate = RateGate(self.ctrl_rate_hz)
        self.last_command = CommandVector()

    def process_imu(self, pkt: MeasurementPacket, dt: float) -> None:
        self.estimator.predict(dt, pkt)

    def process_measurement(self, pkt: MeasurementPacket) -> None:
        self.estimator.update(pkt)

    def update_control(self, truth_state, dt: float) -> CommandVector:
        est: EstimatorState = self.estimator.get_estimated_state()
        mode = self.modes.update(truth_state, est)
        self.last_command = self.controller.update(est, mode, dt)
        return self.last_command
