from __future__ import annotations

from active_rocket_sim.actuators.gimbal import GimbalActuator
from active_rocket_sim.aero.tvc import TVCEffector
from active_rocket_sim.core.events import EventDetector
from active_rocket_sim.core.logging import FlightLogger
from active_rocket_sim.core.state import ActuatorState, EnvironmentState, FlightLogEntry, Vec3
from active_rocket_sim.gnc.flight_computer import FlightComputer
from active_rocket_sim.plant.mass_properties import SimpleBurnModel
from active_rocket_sim.plant.rocketpy_adapter import RocketPyAdapter
from active_rocket_sim.sensors.baro import Barometer
from active_rocket_sim.sensors.gnss import GNSS
from active_rocket_sim.sensors.imu import IMUSensor


def run(duration_s: float = 8.0) -> FlightLogger:
    burn = SimpleBurnModel(dry_mass_kg=12.0, prop_mass_kg=4.0, burn_time_s=2.5)
    env = EnvironmentState(wind_inertial_ms=Vec3(1.0, 0.0, 0.0))
    plant = RocketPyAdapter(burn_model=burn, thrust_n=320.0, dt_s=0.002, env=env)
    state = plant.initial_state()

    imu = IMUSensor()
    baro = Barometer()
    gnss = GNSS()
    fc = FlightComputer()
    act = GimbalActuator()
    event_detector = EventDetector()
    tvc = TVCEffector(thrust_n=320.0, nozzle_pos_body_m=Vec3(0.0, 0.0, -1.2))

    logger = FlightLogger()
    actuator_state = ActuatorState()

    while state.t < duration_s:
        if fc.imu_gate.due(state.t):
            fc.process_imu(imu.sample(state), plant.dt_s)
        if int(state.t * 50) != int((state.t - plant.dt_s) * 50):
            fc.process_measurement(baro.sample(state))
        if int(state.t * 10) != int((state.t - plant.dt_s) * 10):
            fc.process_measurement(gnss.sample(state))

        cmd = fc.last_command
        if fc.ctrl_gate.due(state.t):
            cmd = fc.update_control(state, plant.dt_s)

        actuator_state = act.propagate(cmd, actuator_state, plant.dt_s)
        state, per_effector, burn_remaining = plant.step(state, [lambda s: tvc.compute(s, actuator_state)])
        event_detector.update(state, burn_remaining)

        logger.append(
            FlightLogEntry(
                t=state.t,
                mode=fc.modes.mode,
                state=state,
                estimate=fc.estimator.get_estimated_state(),
                command=cmd,
                actuator=actuator_state,
                per_effector=per_effector,
            )
        )

    return logger


if __name__ == "__main__":
    log = run()
    log.write_jsonl("artifacts/tvc_subscale.jsonl")
    print(f"wrote {len(log.entries)} samples")
