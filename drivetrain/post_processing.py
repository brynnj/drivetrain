"""Build one self-contained HTML report from a simulation."""

from pathlib import Path

import numpy as np
import plotly.graph_objects as go
import plotly.io as pio
from plotly.subplots import make_subplots


def compute_distance(t, v):
    """Trapezoidal integration of vehicle speed, in meters."""
    t, v = np.asarray(t), np.asarray(v)
    if t.shape != v.shape:
        raise ValueError("time and speed arrays must have the same shape")
    return np.r_[0.0, np.cumsum((v[1:] + v[:-1]) * np.diff(t) / 2.0)] if t.size else np.array([])


def dynamic_figure(distance, v, acceleration, omega_w):
    fig = make_subplots(rows=2, cols=1, specs=[[{"secondary_y": True}], [{}]],
                        subplot_titles=("Vehicle and Wheel Speed", "Acceleration"))
    fig.add_trace(go.Scatter(x=distance, y=v, name="Vehicle Speed [m/s]"), row=1, col=1)
    fig.add_trace(go.Scatter(x=distance, y=omega_w * 60 / (2 * np.pi),
                             name="Wheel Speed [RPM]", line=dict(dash="dash")),
                  row=1, col=1, secondary_y=True)
    fig.add_trace(go.Scatter(x=distance, y=acceleration, name="Acceleration [m/s²]"), row=2, col=1)
    fig.update_yaxes(title_text="Vehicle Speed [m/s]", row=1, col=1)
    fig.update_yaxes(title_text="Wheel Speed [RPM]", row=1, col=1, secondary_y=True)
    fig.update_yaxes(title_text="Acceleration [m/s²]", row=2, col=1)
    fig.update_xaxes(title_text="Distance [m]", row=2, col=1)
    fig.update_layout(title="Dynamic Performance", height=650, hovermode="x unified")
    return fig


def force_figure(distance, tire_force, resistance):
    fig = go.Figure()
    for values, label in (
        (tire_force, "Tire Force [N]"),
        (resistance, "Resistance [N]"),
        (tire_force - resistance, "Net Vehicle Force [N]"),
    ):
        fig.add_trace(go.Scatter(x=distance, y=values, name=label))
    fig.update_layout(title="Forces", xaxis_title="Distance [m]", yaxis_title="Force [N]",
                      height=500, hovermode="x unified")
    return fig


def torque_figure(distance, torque_stages):
    fig = go.Figure()
    for stage in torque_stages[0]:
        values = [step[stage] for step in torque_stages]
        fig.add_trace(go.Scatter(x=distance, y=values, name=stage.replace("_", " ").title()))
    fig.update_layout(title="Driveline Torques", xaxis_title="Distance [m]",
                      yaxis_title="Torque [N·m]", height=600, hovermode="x unified")
    return fig


def cvt_figure(distance, result):
    engine_rpm = result["omega_e"]
    ratio = result["i_cvt"]
    belt_rpm = result["omega_w"] * ratio * result["meta"]["total_ratio"] * 60 / (2 * np.pi)
    mode_value = [{"free": 0, "slipping": 1, "locked": 2}[mode] for mode in result["engagement_mode"]]
    fig = make_subplots(rows=3, cols=1,
                        subplot_titles=("Engine and Belt Speeds", "CVT Ratio", "Engagement State"))
    fig.add_trace(go.Scatter(x=distance, y=engine_rpm, name="Engine [RPM]"), row=1, col=1)
    fig.add_trace(go.Scatter(x=distance, y=belt_rpm, name="Driven Belt Speed at Primary [RPM]",
                             line=dict(dash="dash")), row=1, col=1)
    fig.add_hline(y=result["meta"]["omega_launch"], row=1, col=1, line_dash="dot",
                  annotation_text="Engagement RPM")
    fig.add_hline(y=result["meta"]["omega_pp"], row=1, col=1, line_dash="dot",
                  annotation_text="Peak-Power RPM")
    fig.add_trace(go.Scatter(x=distance, y=ratio, name="CVT Ratio"), row=2, col=1)
    fig.add_trace(go.Scatter(x=distance, y=mode_value, name="Engagement State"), row=3, col=1)
    fig.update_yaxes(title_text="RPM", row=1, col=1)
    fig.update_yaxes(title_text="Ratio", row=2, col=1)
    fig.update_yaxes(title_text="State", tickvals=[0, 1, 2],
                     ticktext=["Free", "Slipping", "Locked"], range=[-0.2, 2.2], row=3, col=1)
    fig.update_xaxes(title_text="Distance [m]", row=3, col=1)
    fig.update_layout(title="CVT Behavior", height=850, hovermode="x unified")
    return fig


def tire_figure(distance, result):
    tangent_speed = result["omega_w"] * result["meta"]["tire_radius"]
    fig = make_subplots(rows=2, cols=1,
                        subplot_titles=("Wheel Tangent Speed vs Ground Speed", "Tire Force"))
    fig.add_trace(go.Scatter(x=distance, y=result["v"], name="Ground Speed [m/s]"), row=1, col=1)
    fig.add_trace(go.Scatter(x=distance, y=tangent_speed, name="Wheel Tangent Speed [m/s]"), row=1, col=1)
    for key, label, dash in (
        ("F_x", "Tire Force Supplied [N]", None),
        ("F_x_steady", "Steady Tire Force [N]", "dot"),
        ("F_x_peak", "Peak Traction [N]", "dash"),
    ):
        fig.add_trace(go.Scatter(x=distance, y=result[key], name=label,
                                 line=dict(dash=dash) if dash else None), row=2, col=1)
    fig.update_yaxes(title_text="Speed [m/s]", row=1, col=1)
    fig.update_yaxes(title_text="Force [N]", row=2, col=1)
    fig.update_xaxes(title_text="Distance [m]", row=2, col=1)
    fig.update_layout(title="Wheel Slip and Traction", height=800, hovermode="x unified")
    return fig


def generate_all_plots(result, output_dir="plots"):
    """Write every plot into one HTML file and return its path."""
    distance = compute_distance(result["t"], result["v"])
    figures = (
        dynamic_figure(distance, result["v"], result["a"], result["omega_w"]),
        force_figure(distance, result["F_x"], result["F_resist"]),
        torque_figure(distance, result["torque_stages"]),
        cvt_figure(distance, result),
        tire_figure(distance, result),
    )
    sections = [
        f"<section>{pio.to_html(fig, full_html=False, include_plotlyjs=(index == 0))}</section>"
        for index, fig in enumerate(figures)
    ]
    path = Path(output_dir) / "acceleration_results.html"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Acceleration Results</title>"
        "<style>body{font-family:Arial,sans-serif;max-width:1400px;margin:24px auto;padding:0 20px}"
        "section{margin:40px 0}</style></head><body><h1>Acceleration Results</h1>"
        + "".join(sections) + "</body></html>", encoding="utf-8"
    )
    return path
