"""Straight-line CVT, wheel, and vehicle dynamics."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .config import EventConfig, VehicleConfig


@dataclass
class StepResult:
    """Accelerations and quantities used by the integrator and report."""

    dv_dt: float
    domega_w_dt: float
    domega_e_dt: float
    engine_rpm: float
    F_x: float
    F_x_steady: float
    tire_relaxation_speed: float
    F_x_peak: float
    F_resist: float
    i_cvt: float
    torque_stages: dict[str, float]


class AccelerationModel:
    """Compute accelerations at one instant from vehicle and event settings."""
    def __init__(self, vehicle_config: VehicleConfig, event_config: EventConfig | None = None) -> None:
        """Event settings override vehicle defaults."""
        if event_config is not None:
            self.cfg = vehicle_config.merge(event_config.to_dict())
        else:
            self.cfg = vehicle_config

        self.stages = self.cfg.driveline.stages
        self.omega_idle = float(self.cfg.engine.get("omega_idle", 1800.0))

        self.total_ratio = float(np.prod([stage["ratio"] for stage in self.stages]))
        self.stage_torque_factor = float(np.prod([stage["ratio"] * stage["eta"] for stage in self.stages]))
        self.J_downstream = self.compute_reflected_inertia()
        self._build_curves()

    def _build_curves(self) -> None:
        """Keep the curve points as arrays for interpolation."""
        torque_data = self.cfg.engine.torque_curve
        self.omega_e_vals = np.array([pt["omega"] for pt in torque_data])
        self.T_e_vals = np.array([pt["torque"] for pt in torque_data])

        tire_cfg = self.cfg.tire
        self.burckhardt_B = tire_cfg.burckhardt.B
        self.burckhardt_C = tire_cfg.burckhardt.C
        self.burckhardt_D = tire_cfg.burckhardt.D
        self.relaxation_length = tire_cfg.relaxation_length
        peak_slip = max(np.log(self.burckhardt_B * self.burckhardt_C / self.burckhardt_D) / self.burckhardt_C, 0.0)
        self.peak_mu = self._burckhardt_mu(peak_slip)

        cvt_data = self.cfg.cvt.efficiency_map
        self.i_cvt_vals = np.array([pt["ratio"] for pt in cvt_data])
        self.eta_cvt_vals = np.array([pt["eta"] for pt in cvt_data])

    def engine_torque(self, omega_e: float) -> float:
        """Engine torque at RPM; stay at the curve endpoints outside its range."""
        return float(np.interp(omega_e, self.omega_e_vals, self.T_e_vals))

    def cvt_efficiency(self, i_cvt: float) -> float:
        """Compute CVT efficiency at given ratio."""
        return float(np.interp(i_cvt, self.i_cvt_vals, self.eta_cvt_vals))

    def _rpm_from_rad_s(self, omega: float) -> float:
        return omega * 60.0 / (2.0 * np.pi)

    def _rad_s_from_rpm(self, omega_rpm: float) -> float:
        return omega_rpm * (2.0 * np.pi) / 60.0

    def compute_slip(self, v: float, omega_w: float, r_t: float) -> float:
        """Normalized longitudinal slip, regularized near zero speed."""
        wheel_tangent_speed = r_t * omega_w
        denominator = max(abs(v), abs(wheel_tangent_speed), 0.1)
        return (wheel_tangent_speed - v) / denominator

    def _burckhardt_mu(self, slip_magnitude: float) -> float:
        """Burckhardt friction coefficient for nonnegative longitudinal slip."""
        mu = self.burckhardt_B * (1.0 - np.exp(-self.burckhardt_C * slip_magnitude)) - self.burckhardt_D * slip_magnitude
        return max(float(mu), 0.0)

    def compute_reflected_inertia(self) -> float:
        """Sum wheel and fixed-stage inertias at the wheel shaft."""
        J_eq = self.cfg.tire.J_wheel_each * self.cfg.tire.driven_tire_count

        downstream_ratio = self.total_ratio
        for stage in self.stages:
            J_eq += stage.get("J", 0.0) * downstream_ratio**2
            downstream_ratio /= stage["ratio"]
        return J_eq

    def engagement_wheel_speed(self) -> float:
        """Wheel speed where the low-ratio belt speeds match at launch RPM."""
        return self._rad_s_from_rpm(self.cfg.engine.omega_launch) / (
            self.cfg.cvt.i_max * self.total_ratio
        )

    def cvt_ratio(self, omega_w: float, mode: str) -> tuple[float, float]:
        """Return CVT ratio and d(engine speed)/d(wheel speed) when locked."""
        if mode != "locked" or omega_w <= 0.0:
            ratio = self.cfg.cvt.i_max
        else:
            target = self._rad_s_from_rpm(self.cfg.engine.omega_pp) / (omega_w * self.total_ratio)
            ratio = float(np.clip(target, self.cfg.cvt.i_min, self.cfg.cvt.i_max))
        speed_ratio = ratio * self.total_ratio
        shifting = mode == "locked" and self.cfg.cvt.i_min < ratio < self.cfg.cvt.i_max
        return ratio, (0.0 if shifting else speed_ratio)

    def step(
        self,
        v: float,
        omega_w: float,
        distance: float = 0.0,
        omega_e: float | None = None,
        tire_force: float | None = None,
        mode: str = "free",
    ) -> StepResult:
        """Return state derivatives and values needed for the report."""
        cfg = self.cfg
        if omega_e is None:
            omega_e = self._rad_s_from_rpm(self.omega_idle)
        r_t = cfg.tire.r_t
        m_vehicle = cfg.vehicle.m_total
        m_trailer = cfg.load.get("trailer_mass", 0.0)
        m_total = m_vehicle + m_trailer
        g = cfg.environment.gravity
        rho = cfg.environment.get("air_density", cfg.environment.rho)
        CdA = cfg.vehicle.Cd * cfg.vehicle.frontal_area
        C_rr = cfg.vehicle.C_rr
        grade = cfg.environment.grade
        brake_force_per_foot = cfg.load.get("brake_force_per_foot", 0.0)
        # Each driven tire carries its configured share of vehicle weight.
        N = (
            cfg.tire.normal_load_fraction_each
            * cfg.tire.driven_tire_count
            * m_vehicle
            * g
        )

        F_drag = 0.5 * rho * CdA * v**2
        F_rolling = C_rr * m_total * g
        F_grade = m_total * g * np.sin(np.radians(grade))
        F_brake = brake_force_per_foot * max(distance, 0.0) / 0.3048
        F_resist = F_drag + F_rolling + F_grade + F_brake

        i_cvt, h = self.cvt_ratio(omega_w, mode)
        speed_ratio = i_cvt * self.total_ratio
        if mode == "locked":
            omega_e = speed_ratio * omega_w
        omega_e_rpm = self._rpm_from_rad_s(omega_e)

        J_downstream = self.J_downstream
        F_peak = self.peak_mu * N
        slip_ratio = self.compute_slip(v, omega_w, r_t)
        F_x_steady = float(np.sign(slip_ratio) * self._burckhardt_mu(abs(slip_ratio)) * N)
        F_x = float(tire_force if tire_force is not None else 0.0)
        tire_relaxation_speed = max(abs(v), abs(r_t * omega_w), 0.5)

        # 4. In free rev there is no belt torque. During slip the engine is
        # held at launch RPM and its full torque passes through the engagement.
        # Once locked, solve the engine and wheel balances together. The h term
        # accounts for engine acceleration caused by a changing CVT ratio.
        T_engine = self.engine_torque(omega_e_rpm)
        eta_cvt = self.cvt_efficiency(i_cvt)
        wheel_torque_factor = i_cvt * eta_cvt * self.stage_torque_factor
        engine_inertia = cfg.driveline.J_engine
        engine_contribution = wheel_torque_factor * engine_inertia * h if mode == "locked" else 0.0
        J_effective = J_downstream + engine_contribution

        if mode == "free":
            T_belt = 0.0
            omega_e_dot = T_engine / engine_inertia
            domega_w_dt = -r_t * F_x / J_downstream
        elif mode == "slipping":
            T_belt = T_engine
            omega_e_dot = 0.0
            domega_w_dt = (wheel_torque_factor * T_belt - r_t * F_x) / J_downstream
        else:
            domega_w_dt = (wheel_torque_factor * T_engine - r_t * F_x) / J_effective
            omega_e_dot = h * domega_w_dt
            T_belt = T_engine - engine_inertia * omega_e_dot

        torque_stages = {
            "engine": T_engine,
            "after_engagement": T_belt,
            "wheel": T_belt * wheel_torque_factor,
        }

        dv_dt = (F_x - F_resist) / m_total

        # A brake can hold the vehicle at rest but cannot push it backward.
        if v <= 0.0 and dv_dt < 0.0:
            dv_dt = 0.0

        return StepResult(
            dv_dt=dv_dt,
            domega_w_dt=domega_w_dt,
            domega_e_dt=omega_e_dot,
            engine_rpm=omega_e_rpm,
            F_x=F_x,
            F_x_steady=F_x_steady,
            tire_relaxation_speed=tire_relaxation_speed,
            F_x_peak=F_peak,
            F_resist=F_resist,
            i_cvt=i_cvt,
            torque_stages=torque_stages,
        )
