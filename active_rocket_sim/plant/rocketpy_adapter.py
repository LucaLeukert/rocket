from __future__ import annotations

import math
from typing import Callable, Iterable

from active_rocket_sim.core.frames import quat_from_omega_dt, quat_multiply, rotate_body_to_inertial
from active_rocket_sim.core.state import EnvironmentState, ForceMoment, PlantState, Quaternion, Vec3
from active_rocket_sim.plant.force_moment_sum import sum_force_moment
from active_rocket_sim.plant.mass_properties import SimpleBurnModel


class RocketPyAdapter:
    """Truth-model core adapter with generic effector force/moment injection."""

    def __init__(
        self,
        burn_model: SimpleBurnModel,
        thrust_n: float,
        dt_s: float = 0.002,
        env: EnvironmentState | None = None,
    ):
        self.burn_model = burn_model
        self.thrust_n = thrust_n
        self.dt_s = dt_s
        self.env = env or EnvironmentState()

    def initial_state(self) -> PlantState:
        mp, _, _ = self.burn_model.at(0.0)
        return PlantState(
            t=0.0,
            position_i_m=Vec3(),
            velocity_i_ms=Vec3(),
            omega_b_rads=Vec3(),
            attitude_bi=Quaternion(),
            mass=mp,
            environment=self.env,
        )

    def step(
        self,
        state: PlantState,
        effectors: Iterable[Callable[[PlantState], ForceMoment]],
    ) -> tuple[PlantState, list[ForceMoment], float]:
        dt = self.dt_s
        next_t = state.t + dt
        mp, _mdot, burn_remaining = self.burn_model.at(next_t)

        thrust = self.thrust_n if burn_remaining > 0 else 0.0
        baseline = ForceMoment(force_b_n=Vec3(0.0, 0.0, thrust), source="motor")

        v_rel = state.velocity_i_ms - self.env.wind_inertial_ms
        q = 0.5 * self.env.rho_kgm3 * v_rel.norm() ** 2
        cd = 0.5
        area = 0.01
        drag_n = cd * area * q
        drag_dir = Vec3(0.0, 0.0, -1.0 if state.velocity_i_ms.z >= 0 else 1.0)
        drag = ForceMoment(force_b_n=drag_dir * drag_n, source="drag")

        contributions = [baseline, drag]
        contributions.extend(e(state) for e in effectors)
        total = sum_force_moment(contributions)

        f_i = rotate_body_to_inertial(state.attitude_bi, total.force_b_n)
        f_i = Vec3(f_i.x, f_i.y, f_i.z - mp.mass_kg * self.env.gravity_ms2)

        acc_i = f_i * (1.0 / mp.mass_kg)
        new_vel = state.velocity_i_ms + acc_i * dt
        new_pos = state.position_i_m + new_vel * dt

        ix, iy, iz = mp.inertia_diag_kgm2.x, mp.inertia_diag_kgm2.y, mp.inertia_diag_kgm2.z
        wx, wy, wz = state.omega_b_rads.x, state.omega_b_rads.y, state.omega_b_rads.z
        mx, my, mz = total.moment_b_nm.x, total.moment_b_nm.y, total.moment_b_nm.z
        wdot = Vec3(
            (mx - (iz - iy) * wy * wz) / max(ix, 1e-6),
            (my - (ix - iz) * wx * wz) / max(iy, 1e-6),
            (mz - (iy - ix) * wx * wy) / max(iz, 1e-6),
        )
        new_w = state.omega_b_rads + wdot * dt
        dq = quat_from_omega_dt(new_w, dt)
        new_q = quat_multiply(state.attitude_bi, dq).normalized()

        new_state = PlantState(
            t=next_t,
            position_i_m=new_pos,
            velocity_i_ms=new_vel,
            omega_b_rads=new_w,
            attitude_bi=new_q,
            mass=mp,
            environment=self.env,
            dynamic_pressure_pa=q,
            mach=v_rel.norm() / 343.0,
            on_rail=state.on_rail,
            events=state.events.copy(),
        )
        return new_state, contributions, burn_remaining
