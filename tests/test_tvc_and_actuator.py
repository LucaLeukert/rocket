from active_rocket_sim.actuators.gimbal import GimbalActuator
from active_rocket_sim.aero.tvc import TVCEffector
from active_rocket_sim.core.state import ActuatorState, CommandVector, MassProperties, PlantState, Quaternion, Vec3, EnvironmentState


def test_gimbal_rate_limit():
    act = GimbalActuator(max_rate_rads=1.0)
    state = ActuatorState()
    out = act.propagate(CommandVector(tvc_pitch_rad=0.5), state, 0.1)
    assert 0.099 <= out.tvc_pitch_rad <= 0.101


def test_tvc_generates_moment_from_offset():
    st = PlantState(
        t=0.0,
        position_i_m=Vec3(),
        velocity_i_ms=Vec3(),
        omega_b_rads=Vec3(),
        attitude_bi=Quaternion(),
        mass=MassProperties(10.0, Vec3(), Vec3(1, 1, 1)),
        environment=EnvironmentState(),
    )
    tvc = TVCEffector(thrust_n=100.0, nozzle_pos_body_m=Vec3(0.0, 0.0, -1.0))
    fm = tvc.compute(st, ActuatorState(tvc_yaw_rad=0.1))
    assert abs(fm.force_b_n.x) > 0.0
    assert abs(fm.moment_b_nm.y) > 0.0
