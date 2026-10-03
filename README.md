# Straight-line drivetrain model

This is a small model for exploring how a CVT drivetrain, gearing, tire grip,
and vehicle properties affect acceleration from a stop. Change a few numbers,
run the model, and compare the resulting times and plots.

## Run one case

Install `uv`, then from this folder run:

```text
uv sync
uv run python main.py
```

At the top of `main.py`, choose **one** vehicle and **one** event:

```python
VEHICLE = vehicles.BASE_VEHICLE
EVENT = events.BASE_EVENT
```

The vehicle choices are `BASE_VEHICLE` and `PLANETARY_VEHICLE`. The event
choices are `BASE_EVENT` (asphalt), `DIRT_EVENT`, and `PILOT_TOW_EVENT`.
Each run prints the 100 ft time and creates one interactive report at
`plots/<event name>/acceleration_results.html`.

Edit the files in `drivetrain/configs/vehicles/` to change the car, engine,
CVT, or gearbox. Edit `drivetrain/configs/events/` to change the surface,
load, or run duration. Inputs use SI units except engine speeds (RPM) and the
pilot tow braking ramp (N per foot). `Cd` and `frontal_area` are separate
inputs; drag uses their product.

## What the model does

The engine follows an input torque-versus-RPM curve. At launch it revs to the
engagement speed, drives through a slipping CVT, then locks to the wheel side
when their belt speeds match. Once locked, the CVT selects a ratio intended to
hold the engine near peak-power RPM, within its ratio limits. Engine and wheel
inertias contribute to the acceleration calculation.

Vehicle speed and wheel speed evolve separately, so the wheels can spin faster
than the car travels. A Burckhardt slip curve sets the tire force target, and
tire relaxation makes force build over distance. The four driven tires share
one wheel speed and each carries one quarter of the car's weight. Drag,
rolling resistance, grade, and event loads oppose motion.

This is a **comparison model**, not a calibrated prediction of a particular
car. It does not model load transfer, a finite CVT shift rate, or belt torque
capacity. `definition.txt` has the equations and further assumptions.

## Try a study

Two ready-to-edit examples are:

```text
uv run python gear_ratio_sweep.py
uv run python mass_inertia_sensitivity.py
```

The first scales both gearbox stages to sweep total reduction from 50% to
150% of baseline, then plots 100 ft and 1000 ft times. The second changes each
mass or inertia value by ±10% and plots its effect on 100 ft time. Each script
writes one HTML file under `plots/studies/`; change the constants near its top
to try another range or event.

Other questions to explore: Which CVT ratio limits minimize 100 ft time? How
does a dirt surface change the best gearing? How much do drag area, wheel
inertia, or vehicle mass matter at 100 ft versus 1000 ft? For a new study,
import `run(vehicle, event)` from `drivetrain.simulation`. It returns result
arrays without writing a report; `crossing_time` extracts a distance time.
