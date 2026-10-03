from ...config import VehicleConfig

BASE_VEHICLE = VehicleConfig(
    {
        "vehicle": {
            "name": "MBR Dev",
            "m_total": 150,
            "Cd": 0.8,
            "frontal_area": 3.0,
            "C_rr": 0.014,
        },
        "tire": {
            "r_t": 0.34,
            "J_wheel_each": 0.25,
            "normal_load_fraction_each": 0.25,
            "driven_tire_count": 4,
            "burckhardt": {"B": 1.2801, "C": 23.99, "D": 0.52},
            "relaxation_length": 0.2,
        },
        "engine": {
            "omega_idle": 1800.0, # RPM
            "torque_curve": [
                {"omega": 1000.0, "torque": 0},
                {"omega": 1800.0, "torque": 10.74},
                {"omega": 2400.0, "torque": 25.77},
                {"omega": 3150.0, "torque": 21.48},
                {"omega": 3800, "torque": 10.74},
                {"omega": 3900, "torque": 0},  # Restrictor fade after 3800 RPM
            ],
            "omega_launch": 2400.0,
            "omega_pp": 3150.0,
        },
        "cvt": {
            "i_min": 0.4,
            "i_max": 2.9,
            "efficiency_map": [
                {"ratio": 0.5, "eta": 0.93},
                {"ratio": 1.0, "eta": 0.96},
                {"ratio": 2.5, "eta": 0.92},
            ],
        },
        "driveline": {
            "stages": [
                {"name": "input_to_intermediate", "ratio": 3.3, "eta": 0.98, "J": 0.003},
                {"name": "intermediate_to_output", "ratio": 3.3, "eta": 0.98, "J": 0.003},
            ],
            "J_engine": 0.01,
        },
        "environment": {
            "rho": 1.225,
            "gravity": 9.81,
        },
    }
)
