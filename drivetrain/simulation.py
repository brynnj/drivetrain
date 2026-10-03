"""Advance the vehicle model and collect data for the HTML report."""

from __future__ import annotations

import numpy as np

from .config import EventConfig, VehicleConfig
from .dynamics import AccelerationModel, StepResult


def _event_within_step(model, mode, omega_e, omega_w, rates, duration):
    """Find the next free → slip → lock transition within one time step."""
    launch_speed = model._rad_s_from_rpm(model.cfg.engine.omega_launch)
    match_speed = model.engagement_wheel_speed()

    if mode == "free" and rates.domega_e_dt > 0 and omega_e <= launch_speed:
        time = (launch_speed - omega_e) / rates.domega_e_dt
        if time <= duration:
            return max(time, 0.0), "slipping"
    if mode == "slipping" and rates.domega_w_dt > 0 and omega_w <= match_speed:
        time = (match_speed - omega_w) / rates.domega_w_dt
        if time <= duration:
            return max(time, 0.0), "locked"
    if mode == "locked" and rates.domega_w_dt < 0 and omega_w >= match_speed:
        time = (match_speed - omega_w) / rates.domega_w_dt
        if 0.0 <= time <= duration:
            return time, "slipping"
    return duration, None


def _integrate(
    model: AccelerationModel,
    t_end: float,
    dt: float,
    stop_when_stalled: bool,
) -> dict:
    """Integrate speed in m/s, angular speeds in rad/s, and distance in meters."""
    history = {name: [] for name in (
        "t", "distance", "v", "omega_w", "omega_e", "engagement_mode",
        "a", "i_cvt", "F_x", "F_x_steady", "F_x_peak",
        "F_resist", "torque_stages",
    )}
    t = distance = v = omega_w = 0.0
    omega_e = model._rad_s_from_rpm(model.omega_idle)
    launch_speed = model._rad_s_from_rpm(model.cfg.engine.omega_launch)
    mode = "free" if omega_e < launch_speed else "slipping"
    tire_force = 0.0
    stationary_time = 0.0

    while t <= t_end:
        result = model.step(v, omega_w, distance, omega_e, tire_force, mode)
        sample = {
            "t": t, "distance": distance, "v": v, "omega_w": omega_w,
            "omega_e": result.engine_rpm, "engagement_mode": mode,
            "a": result.dv_dt, "i_cvt": result.i_cvt,
            "F_x": result.F_x, "F_x_steady": result.F_x_steady,
            "F_x_peak": result.F_x_peak,
            "F_resist": result.F_resist, "torque_stages": result.torque_stages,
        }
        for name, value in sample.items():
            history[name].append(value)

        # Split a step at engagement or lock so neither angular speed jumps.
        remaining = dt
        rates: StepResult = result
        while remaining > 1e-12:
            step_dt, next_mode = _event_within_step(model, mode, omega_e, omega_w, rates, remaining)
            v_next = max(0.0, v + rates.dv_dt * step_dt)
            omega_w = max(0.0, omega_w + rates.domega_w_dt * step_dt)
            if mode == "locked":
                ratio, _ = model.cvt_ratio(omega_w, "locked")
                omega_e = ratio * model.total_ratio * omega_w
            elif mode == "slipping":
                omega_e = launch_speed
            else:
                omega_e += rates.domega_e_dt * step_dt
            decay = np.exp(-rates.tire_relaxation_speed * step_dt / model.relaxation_length)
            tire_force = rates.F_x_steady + (tire_force - rates.F_x_steady) * decay
            distance += 0.5 * (v + v_next) * step_dt
            v = v_next
            remaining -= step_dt
            if next_mode is None:
                break
            mode = next_mode
            if mode == "locked":
                omega_w = model.engagement_wheel_speed()
            omega_e = launch_speed
            rates = model.step(v, omega_w, distance, omega_e, tire_force, mode)

        t += dt
        stationary_time = stationary_time + dt if v <= 1e-4 else 0.0
        if stop_when_stalled and stationary_time >= 1.0:
            break

    result = {
        name: values if name == "torque_stages" else np.array(values)
        for name, values in history.items()
    }
    result["meta"] = {
        "omega_launch": model.cfg.engine.omega_launch,
        "omega_pp": model.cfg.engine.omega_pp,
        "total_ratio": model.total_ratio,
        "tire_radius": model.cfg.tire.r_t,
    }
    return result


def run(vehicle_config: VehicleConfig, event_config: EventConfig) -> dict:
    """Run one vehicle/event pair. Returns data; report generation is optional."""
    settings = event_config.simulation
    model = AccelerationModel(vehicle_config, event_config)
    return _integrate(
        model,
        t_end=settings.t_end,
        dt=settings.dt,
        stop_when_stalled=settings.stop_when_stalled,
    )


def crossing_time(t, distance, target):
    """First time a monotonic distance trace reaches the target."""
    index = int(np.searchsorted(distance, target))
    if index == len(distance):
        return None
    if index == 0:
        return float(t[0])
    return float(np.interp(target, distance[index - 1:index + 1], t[index - 1:index + 1]))
