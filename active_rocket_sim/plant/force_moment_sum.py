from __future__ import annotations

from typing import Iterable
from active_rocket_sim.core.state import ForceMoment, Vec3


def sum_force_moment(items: Iterable[ForceMoment]) -> ForceMoment:
    total_f = Vec3()
    total_m = Vec3()
    for item in items:
        total_f = total_f + item.force_b_n
        total_m = total_m + item.moment_b_nm
    return ForceMoment(force_b_n=total_f, moment_b_nm=total_m, source="total")
