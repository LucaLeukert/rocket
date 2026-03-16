from __future__ import annotations

from dataclasses import dataclass
from active_rocket_sim.core.state import MassProperties, Vec3


@dataclass
class SimpleBurnModel:
    dry_mass_kg: float
    prop_mass_kg: float
    burn_time_s: float

    def at(self, t: float) -> tuple[MassProperties, float, float]:
        frac = max(0.0, min(1.0, 1.0 - t / self.burn_time_s)) if self.burn_time_s > 0 else 0.0
        prop = self.prop_mass_kg * frac
        mass = self.dry_mass_kg + prop
        com_shift = 0.2 * frac
        inertia = Vec3(0.8 * mass, 0.8 * mass, 0.1 * mass)
        mp = MassProperties(mass_kg=mass, com_body_m=Vec3(0.0, 0.0, -com_shift), inertia_diag_kgm2=inertia)
        mdot = self.prop_mass_kg / self.burn_time_s if t < self.burn_time_s and self.burn_time_s > 0 else 0.0
        burn_remaining = max(0.0, self.burn_time_s - t)
        return mp, mdot, burn_remaining
