"""Estimate each mass/inertia effect on 100 ft time using ±10% changes."""

from pathlib import Path

import plotly.graph_objects as go

from drivetrain.configs.events import BASE_EVENT
from drivetrain.configs.vehicles import BASE_VEHICLE
from drivetrain.simulation import crossing_time, run


CHANGE = 0.10
OUTPUT = Path(__file__).parent / "plots" / "studies" / "mass_inertia_sensitivity.html"


def time_100ft(vehicle):
    result = run(vehicle, BASE_EVENT)
    time = crossing_time(result["t"], result["distance"], 30.48)
    if time is None:
        raise ValueError("100 ft was not reached; increase event simulation t_end")
    return time


def main():
    # Each entry is one independent config value; trailer mass is zero in this event.
    values = (
        ("Vehicle mass", lambda c: c.vehicle, "m_total"),
        ("Engine inertia", lambda c: c.driveline, "J_engine"),
        ("Each wheel inertia", lambda c: c.tire, "J_wheel_each"),
        ("First stage inertia", lambda c: c.driveline.stages[0], "J"),
        ("Second stage inertia", lambda c: c.driveline.stages[1], "J"),
    )
    names, sensitivities = [], []
    for name, select, key in values:
        times = []
        for factor in (1 - CHANGE, 1 + CHANGE):
            vehicle = BASE_VEHICLE.copy()
            component = select(vehicle)
            component[key] *= factor
            times.append(time_100ft(vehicle))
        names.append(name)
        sensitivities.append((times[1] - times[0]) / (200 * CHANGE))

    fig = go.Figure(go.Bar(x=sensitivities, y=names, orientation="h"))
    fig.update_layout(title="100 ft Time Sensitivity (central ±10% difference)",
                      xaxis_title="Change in 100 ft time [s per 1% parameter increase]",
                      yaxis=dict(autorange="reversed"), height=450)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(OUTPUT, include_plotlyjs=True)
    for name, sensitivity in zip(names, sensitivities):
        print(f"{name}: {sensitivity:+.5f} s per 1%")
    print(f"Saved {OUTPUT}")


if __name__ == "__main__":
    main()
