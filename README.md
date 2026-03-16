# Active Rocket Sim

This repository implements an end-to-end closed-loop active rocket simulation stack with:

- 6-DoF truth-plant integration scaffold (`RocketPyAdapter`) with variable-mass model.
- Generic per-step effector injection (`ForceMoment` contributions from TVC and future effectors).
- Multi-rate avionics timing (IMU/control at fixed rates, baro/GNSS slower sampled updates).
- Sensor pipeline (`IMU`, `Barometer`, `GNSS`), estimator, mode manager, controller, and actuator dynamics.
- TVC effector and gimbal actuator with saturation/rate/deadband.
- SITL-friendly packet codec and JSONL deterministic replay support.
- End-to-end runnable example (`active_rocket_sim/examples/tvc_subscale.py`).
- Unit tests that validate closed-loop execution and core actuator/effector behavior.

Run:

```bash
python -m active_rocket_sim.examples.tvc_subscale
pytest
```
