from .base import BASE_EVENT

# Representative Honda Pilot curb mass, approximated for a simple tow case.
HONDA_PILOT_MASS_KG = 2050.0

PILOT_TOW_EVENT = BASE_EVENT.merge(
    {
        "event": {
            "name": "pilot_tow",
            "description": "Tow a Honda Pilot with fixed CVT ratio and distance-ramped braking.",
        },
        "load": {
            "trailer_mass": HONDA_PILOT_MASS_KG,
            "brake_force_per_foot": 100.0,
        },
        "cvt": {
            "i_min": 2.9,
            "i_max": 2.9,
        },
        "simulation": {
            "t_end": 120.0,
            "stop_when_stalled": True,
        },
    }
)
