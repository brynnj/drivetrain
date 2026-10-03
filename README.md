# Straight-line drivetrain model

This is a small model for straight-line driving. Includes a simple engagement model, ideal CVT shifting, wheel slip / traction modeling, towing configs, hill climbing, and other losses (frictions/efficiencies/drag).

All the inputs are guesses/placeholders

## Run one case

Install `uv`, then from this folder run:

```text
uv sync
uv run python main.py
```

At the top of `main.py`, choose a vehicle and an event:

```python
VEHICLE = vehicles.BASE_VEHICLE
EVENT = events.BASE_EVENT
```

Each run prints the 100 ft time and creates a report at
`plots/<event name>/acceleration_results.html`, you'll probably want to extend/adapt the outputs for different cases.

Edit/create files in `drivetrain/configs/vehicles/` to change the car, engine,
CVT, or gearbox. Edit `drivetrain/configs/events/` to change the surface,
load, or run duration. Inputs use SI units except engine speeds (RPM) and the
pilot tow braking ramp (N per foot).

## What the model does

The engine follows an input torque-speed curve. At launch it revs to the
CVT engagement speed, holds there while the belt slips at the initial CVT ratio and delivers torque to the drivetrain, then locks to the rest of the drivetrain when the belt speeds match. Once locked, the CVT selects a ratio intended to hold the engine near peak-power RPM, within its ratio limits.

Vehicle speed and wheel speed are separate states, so the wheels can spin faster
than the car travels, which is needed for slip modeling. This uses a Burckhardt tire model
but with all estimated parameters, this is probably the hardest thing to populate.
There's an additional state for tire elastic force, which makes force build over distance and allows it to relax when it comes to a stop. The four driven tires share
one wheel speed and each carries one quarter of the car's weight. This would be an area for improvement - modeling the overrun/underrun of 4WD and assessing impact on speed would be a good use case.

This is probably most useful as a comparison/trade study tool not a super accurate prediction. `definition.txt` has the equations and further assumptions.

## Try a study

Two example studies are in:

```text
uv run python gear_ratio_sweep.py
uv run python mass_inertia_sensitivity.py
```

The first scales both gearbox stages to sweep total reduction from 50% to
150% of baseline, then plots 100 ft and 1000 ft times. The second changes each
mass or inertia value by ±10% and plots its effect on 100 ft time. Each script
writes one HTML file under `plots/studies/`.

Some ideas for what this model could be used for:
- optimizing a gearbox ratio, demonstrating towing improvements w/ the planetary
- determining sensitivities (i.e. how worthwhile is it to shave 5% off the mass of the driveshafts, how much top speed could be gained with more optimal aero)
- if you expand it to model weight transfer, optimizing accel & towing suspension setups on different terrains
- you could also build a maneuv or endurance type track with distances of full throttle and distances of braking, and then compare different gear ratios for lap times
- could also see if hill climb wants a different ratio
- the endgame version would be to set up each event in comp, relate the times back to points, and optimize the car across all the events so that you maximize predicted points in competition
- everything in this model can probably be done better in CarSim, so if you've figured that out pretty well this might be obsolete. this can run quickly though for large sweeps and you could even parallelize it
- if you could get really good traction data, you could even optimize weight (and weight transfer) such that you get close to the optimal slip ratio on softer terrain. Offroad tires want to slip a bit for max performance
