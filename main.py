"""Choose one vehicle and event here, then run this file."""

from pathlib import Path

from drivetrain.configs import events, vehicles
from drivetrain.post_processing import generate_all_plots
from drivetrain.simulation import crossing_time, run

VEHICLE = vehicles.BASE_VEHICLE
EVENT = events.BASE_EVENT


def main():
    result = run(VEHICLE, EVENT)
    report = generate_all_plots(result, Path(__file__).parent / "plots" / EVENT.event.name)
    time_100ft = crossing_time(result["t"], result["distance"], 30.48)
    summary = f"100 ft: {time_100ft:.2f} s" if time_100ft is not None else "100 ft: not reached"
    print(f"{EVENT.event.name}: {summary}; distance: {result['distance'][-1]:.2f} m")
    print(f"Report: {report}")


if __name__ == "__main__":
    main()
