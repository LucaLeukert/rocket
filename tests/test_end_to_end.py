from active_rocket_sim.examples.tvc_subscale import run


def test_closed_loop_run_produces_log_and_liftoff():
    log = run(1.5)
    assert len(log.entries) > 100
    assert log.entries[-1].state.position_i_m.z > 0.0
