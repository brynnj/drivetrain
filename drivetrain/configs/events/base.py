from ...config import EventConfig

BASE_EVENT = EventConfig(
    {
        "event": {
            "name": "base",
            "description": "Straight-line acceleration from rest on level ground.",
        },
        "load": {
            "trailer_mass": 0.0,
        },
        "environment": {
            "grade": 0.0,
            "air_density": 1.225,
            "gravity": 9.81,
        },
        "simulation": {
            "t_end": 10.0,
            "dt": 0.001,
            "stop_when_stalled": False,
        },
    }
)
