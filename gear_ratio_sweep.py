"""Sweep the total fixed gearbox reduction and plot acceleration times."""

from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from drivetrain.configs.events import BASE_EVENT
from drivetrain.configs.vehicles import BASE_VEHICLE
from drivetrain.simulation import crossing_time, run


SCALES = np.linspace(0.5, 1.5, 11)  # Total reduction relative to baseline
EVENT = BASE_EVENT.merge({"simulation": {"t_end": 45.0}})
OUTPUT = Path(__file__).parent / "plots" / "studies" / "gear_ratio_sweep.html"


def main():
    stages = BASE_VEHICLE.driveline.stages
    baseline_ratio = stages[0]["ratio"] * stages[1]["ratio"]
    ratios, times_100, times_1000 = [], [], []

    for scale in SCALES:
        vehicle = BASE_VEHICLE.copy()
        for stage in vehicle.driveline.stages:
            # Multiplying both stages by √scale multiplies their product by scale.
            stage["ratio"] *= np.sqrt(scale)
        result = run(vehicle, EVENT)
        ratios.append(baseline_ratio * scale)
        times_100.append(crossing_time(result["t"], result["distance"], 30.48))
        times_1000.append(crossing_time(result["t"], result["distance"], 304.8))
        if times_100[-1] is None or times_1000[-1] is None:
            raise ValueError("A distance was not reached; increase EVENT simulation t_end")

    for ratio, t100, t1000 in zip(ratios, times_100, times_1000):
        print(f"{ratio:6.3f} reduction: 100 ft {t100:5.2f} s; 1000 ft {t1000:5.2f} s")

    fig = make_subplots(rows=2, cols=1, shared_xaxes=True,
                        subplot_titles=("100 ft", "1000 ft"))
    for row, times in ((1, times_100), (2, times_1000)):
        fig.add_trace(go.Scatter(x=ratios, y=times, mode="lines+markers",
                                 name=f"{100 if row == 1 else 1000} ft"), row=row, col=1)
        fig.update_yaxes(title_text="Time [s]", row=row, col=1)
    fig.add_vline(x=baseline_ratio, line_dash="dash", line_color="gray")
    fig.update_xaxes(title_text="Total fixed gearbox reduction", row=2, col=1)
    fig.update_layout(title="Gear Ratio Sweep", height=750, showlegend=False)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(OUTPUT, include_plotlyjs=True)
    print(f"Saved {OUTPUT}")


if __name__ == "__main__":
    main()
