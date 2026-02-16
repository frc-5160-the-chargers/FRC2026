"""
Coefficient of drag(or C_D, for short)
can be calculated by dropping something at a set height
"""
from dataclasses import dataclass
import math

import numpy as np
from sleipnir.autodiff import VariableMatrix
from sleipnir.optimization import Problem


@dataclass
class DropTrial:
    # seconds; make sure you use the true timestamps if you're using slowmo
    time_to_ground: float
    # meters
    drop_height: float


mass = 0.5 / 2.205 # kilograms
ball_radius = 5.91 * 0.0254 / 2 # meters
cross_sectional_area = math.pi * (ball_radius ** 2)
g = 9.81 # m/s^2
rho = 1.221 # kg/m^3, density of air
samples = 1000 # Unitless; the number of samples for sleipnir optimization
trials = [
    DropTrial(time_to_ground=7.59 * 0.1, drop_height=9 * 12 * 0.0254)
]


def f(x, C_D):
    v = x[1]
    A = cross_sectional_area
    F_D = 0.5 * rho * (v ** 2) * C_D * A # Force of drag

    return VariableMatrix([[v], [g - F_D / mass]])


if __name__ == '__main__':
    drag_coeff_values = []
    for trial in trials:
        problem = Problem()

        x = problem.decision_variable(2, 1)
        C_D = problem.decision_variable()
        dt = trial.time_to_ground / samples

        # RK4 integration
        x_k = x
        for _ in range(samples):
            k1 = f(x_k, C_D)
            k2 = f(x_k + dt / 2 * k1, C_D)
            k3 = f(x_k + dt / 2 * k2, C_D)
            k4 = f(x_k + dt * k3, C_D)
            x_k += dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        # Initial state must be (position = 0, velocity = 0)
        problem.subject_to(x == np.array([[0], [0]]))
        # Final position must be equal to drop position
        problem.subject_to(x_k[0,0] == trial.drop_height)

        problem.solve(diagnostics=True)
        drag_coeff_values.append(C_D.value())
    print(f"Drag Coefficient: {np.mean(drag_coeff_values)}")
    if len(drag_coeff_values) > 1:
        print(f"Std Dev: {np.std(drag_coeff_values)}")
