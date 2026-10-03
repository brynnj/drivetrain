from .base import BASE_VEHICLE

PLANETARY_VEHICLE = BASE_VEHICLE.merge(
    {
        "vehicle": {
            "name": "MBR Dev + Planetary",
        },
        "driveline": {
            "stages": [
                {"name": "planetary", "ratio": 3.0, "eta": 1.0, "J": 0.0},
                {"name": "input_to_intermediate", "ratio": 3.3, "eta": 0.98, "J": 0.0},
                {"name": "intermediate_to_output", "ratio": 3.3, "eta": 0.98, "J": 0.0},
            ],
        },
    }
)
