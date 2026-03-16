from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, List


@dataclass
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vec3":
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    __rmul__ = __mul__

    def norm(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def cross(self, other: "Vec3") -> "Vec3":
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )


@dataclass
class Quaternion:
    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def normalized(self) -> "Quaternion":
        n = math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        if n == 0:
            return Quaternion()
        return Quaternion(self.w / n, self.x / n, self.y / n, self.z / n)


@dataclass
class MassProperties:
    mass_kg: float
    com_body_m: Vec3
    inertia_diag_kgm2: Vec3


@dataclass
class EnvironmentState:
    rho_kgm3: float = 1.225
    gravity_ms2: float = 9.80665
    wind_inertial_ms: Vec3 = field(default_factory=Vec3)


@dataclass
class PlantState:
    t: float
    position_i_m: Vec3
    velocity_i_ms: Vec3
    omega_b_rads: Vec3
    attitude_bi: Quaternion
    mass: MassProperties
    environment: EnvironmentState
    dynamic_pressure_pa: float = 0.0
    mach: float = 0.0
    on_rail: bool = True
    events: Dict[str, bool] = field(default_factory=dict)


@dataclass
class EstimatorState:
    attitude_bi: Quaternion = field(default_factory=Quaternion)
    omega_b_rads: Vec3 = field(default_factory=Vec3)
    altitude_m: float = 0.0
    vertical_velocity_ms: float = 0.0


@dataclass
class CommandVector:
    tvc_pitch_rad: float = 0.0
    tvc_yaw_rad: float = 0.0


@dataclass
class ActuatorState:
    tvc_pitch_rad: float = 0.0
    tvc_yaw_rad: float = 0.0
    tvc_pitch_rate_rads: float = 0.0
    tvc_yaw_rate_rads: float = 0.0


@dataclass
class MeasurementPacket:
    sensor: str
    t: float
    values: Dict[str, float]


@dataclass
class ForceMoment:
    force_b_n: Vec3 = field(default_factory=Vec3)
    moment_b_nm: Vec3 = field(default_factory=Vec3)
    source: str = "unknown"


@dataclass
class FlightLogEntry:
    t: float
    mode: str
    state: PlantState
    estimate: EstimatorState
    command: CommandVector
    actuator: ActuatorState
    per_effector: List[ForceMoment]
