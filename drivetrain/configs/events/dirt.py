from .base import BASE_EVENT

DIRT_EVENT = BASE_EVENT.merge(
    {
        "event": {
            "name": "dirt",
            "description": "Straight-line acceleration from rest on a low-traction dirt surface.",
        },
        "tire": {
            # Approximate loose-gravel curve; peak is about 0.61 at 24% slip.
            "burckhardt": {"B": 0.65, "C": 18.0, "D": 0.15},
        },
    }
)
