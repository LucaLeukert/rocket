from __future__ import annotations

from dataclasses import dataclass, field

from active_rocket_sim.core.frames import quat_from_omega_dt, quat_multiply
from active_rocket_sim.core.state import EstimatorState, MeasurementPacket, Quaternion, Vec3


@dataclass
class AttitudeComplementaryFilter:
    state: EstimatorState = field(default_factory=EstimatorState)

    def predict(self, dt: float, imu_packet: MeasurementPacket) -> None:
        w = Vec3(imu_packet.values["gx"], imu_packet.values["gy"], imu_packet.values["gz"])
        self.state.omega_b_rads = w
        dq = quat_from_omega_dt(w, dt)
        self.state.attitude_bi = quat_multiply(self.state.attitude_bi, dq).normalized()

    def update(self, pkt: MeasurementPacket) -> None:
        if pkt.sensor == "baro":
            self.state.altitude_m = pkt.values["altitude_m"]
        elif pkt.sensor == "gnss":
            self.state.vertical_velocity_ms = pkt.values.get("vz_ms", self.state.vertical_velocity_ms)

    def get_estimated_state(self) -> EstimatorState:
        return self.state
