"""Physical measurements in SI units, shared by the current controllers.

Edit measurements here, then restart the controller. After double-arm changes,
run PythonMasterMath2.py to regenerate control_matrices2.txt before balancing.
The single and double setups intentionally retain their different measurements.
"""

from dataclasses import dataclass

GRAVITY_M_S2 = 9.81
SAMPLE_PERIOD_S = 1 / 100
ENCODER_COUNTS = 16384  # 14-bit encoders; must agree with the receiver firmware.
DISTANCE_PER_6400_STEPS_M = 0.638175


@dataclass(frozen=True)
class ArmParameters:
    length_m: float
    center_of_mass_m: float
    inertia_kg_m2: float
    mass_kg: float

    @property
    def weight_n(self):
        """Gravitational force in newtons; mass is stored in kilograms."""
        return self.mass_kg * GRAVITY_M_S2


# Single-arm balancing and swing-up use this setup.
SINGLE_ARM = ArmParameters(
    length_m=0.2, center_of_mass_m=0.05,
    inertia_kg_m2=0.003, mass_kg=0.260,
)
SINGLE_CART_MASS_KG = 0.969
SINGLE_CART_DRAG = 0.5
SINGLE_PIVOT_DRAG = 0.0

# Double-arm model generation uses these measurements.
DOUBLE_ARM_1 = ArmParameters(
    length_m=0.2, center_of_mass_m=0.087,
    inertia_kg_m2=0.00123324992, mass_kg=0.207,
)
DOUBLE_ARM_2 = ArmParameters(
    length_m=0.25, center_of_mass_m=0.106,
    inertia_kg_m2=0.00197563667, mass_kg=0.210,
)
DOUBLE_CART_MASS_KG = 0.969 - 0.059
DOUBLE_CART_DRAG = 0.0
DOUBLE_PIVOT_DRAG = 0.0  # Existing model does not model rotational drag correctly.
