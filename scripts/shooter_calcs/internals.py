import math

import numpy as np
from numpy.linalg import norm
from sleipnir.autodiff import VariableMatrix, atan2, hypot
from sleipnir.optimization import ExitStatus, Problem

from shooter_calcs.data_types import ShooterSolution

# Physical characteristics
shooter_height = 20 * 0.0254  # m
g = 9.81  # m/s²
max_shooter_velocity = 20  # m/s
ball_mass = 0.5 / 2.205  # kg
ball_diameter = 5.91 * 0.0254  # m

# Solve settings
N = 40
start_distance = 0.5
end_distance = 22.5
distance_samples = 20
distance_exponent = 2
print_results = False


def lerp(a, b, t):
    """ Performs Linear Interpolation. """
    return a + t * (b - a)


def f(x):
    """
    Apply the drag equation to a velocity.
    """
    # x' = x'
    # y' = y'
    # z' = z'
    # x" = −a_D(v_x)
    # y" = −a_D(v_y)
    # z" = −g − a_D(v_z)
    #
    # where a_D(v) = ½ρv² C_D A / m
    # (see https://en.wikipedia.org/wiki/Drag_(physics)#The_drag_equation)
    rho = 1.204  # kg/m³, density of air
    C_D = 0.4 # drag coefficient
    m = ball_mass
    A = math.pi * ((ball_diameter / 2) ** 2) # cross-sectional area
    drag_force = lambda v: 0.5 * rho * v**2 * C_D * A
    a_D = lambda v: drag_force(v) / m

    v_x = x[3, 0]
    v_y = x[4, 0]
    v_z = x[5, 0]
    return VariableMatrix(
        [[v_x], [v_y], [v_z], [-a_D(v_x)], [-a_D(v_y)], [-g - a_D(v_z)]]
    )


def setup_problem(distance, target_height):
    """
    Set up the problem and any shared constraints between the two solve modes (min and fix vel)
    """
    # Robot initial state ([x, y, z, vx, vy, vz])
    # Here, the "field" is not the blue alliance origin, but the point
    # directly below the hub with a height of 0.
    shooter_wrt_field = np.array(
        [[-distance], [0], [shooter_height], [0], [0], [0]]
    )
    target_wrt_field = np.array(
        [[0], [0], [target_height], [0], [0], [0]]
    )

    problem = Problem()

    # Set up duration decision variables
    T = problem.decision_variable()
    problem.subject_to(T >= 0)
    T.set_value(1)
    dt = T / N

    # Ball state in field frame ([x, y, z, vx, vy, vz])
    X = problem.decision_variable(6, N)

    p = X[:3, :]

    v_x = X[3, :]
    v_y = X[4, :]
    v_z = X[5, :]

    v0_wrt_shooter = X[3:, :1] - shooter_wrt_field[3:, :]

    # Shooter initial position
    problem.subject_to(p[:, :1] == shooter_wrt_field[:3, :])

    # Dynamics constraints - RK4 integration
    h = dt
    for k in range(N - 1):
        x_k = X[:, k]
        x_k1 = X[:, k + 1]

        k1 = f(x_k)
        k2 = f(x_k + h / 2 * k1)
        k3 = f(x_k + h / 2 * k2)
        k4 = f(x_k + h * k3)
        problem.subject_to(x_k1 == x_k + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4))

    # Require final position is in center of target circle
    problem.subject_to(p[:, -1] == target_wrt_field[:3, :])

    # Require the final velocity is at least somewhat downwards by limiting horizontal velocity
    # and requiring negative vertical velocity
    problem.subject_to(v_z[-1] < -1)
    # Max horizontal velocity is 2.5 times the downwards velocity (~21 degrees from horizontal)
    problem.subject_to(hypot(v_x[-1], v_y[-1]) <= v_z[-1] * -5)

    return problem, shooter_wrt_field, target_wrt_field, v0_wrt_shooter, T, X


def compute_results(status, v0_wrt_shooter, T, distance, X):
    if status == ExitStatus.SUCCESS:
        # Initial velocity vector with respect to shooter
        v0 = v0_wrt_shooter.value()
        velocity: float = norm(v0)
        pitch: float = math.atan2(v0[2, 0], math.hypot(v0[0, 0], v0[1, 0]))
        time: float = T.value()

        if print_results:
            print(f"Min velocity solve:")
            print(f"Distance = {distance:.03f} m")
            print(f"Velocity = {velocity:.03f} m/s")
            print(f"Pitch = {np.rad2deg(pitch):.03f}°")
            print(f"Time = {time:.03f}s")

        return ShooterSolution(velocity_mps=velocity, pitch_rad=pitch, time_secs=time, X=X)
    print(f"Infeasible at distance {distance:.03f} m with status {status.name}")
    return None


def fixed_pitch(distance: float, target_height: float, pitch: float, prev_X: VariableMatrix):
    """
    Solve for minimum velocity.
    :returns: A tuple of [True, velocity, pitch, yaw, X] if it succeeds at a solve, and a tuple of[False, 0] if it fails.
    """
    problem, shooter_wrt_field, target_wrt_field, v0_wrt_shooter, T, X = setup_problem(distance, target_height)

    prev_p_x = prev_X[0, :]
    prev_p_y = prev_X[1, :]
    prev_p_z = prev_X[2, :]

    prev_v = prev_X[3:, :]

    p_x = X[0, :]
    p_y = X[1, :]
    p_z = X[2, :]

    v = X[3:, :]

    # Position initial guess is last solve's position
    for k in range(N):
        p_x[k].set_value(prev_p_x[k].value())
        p_y[k].set_value(prev_p_y[k].value())
        p_z[k].set_value(prev_p_z[k].value())

    # Velocity initial guess is last solve's velocity
    for k in range(N):
        v[:, k].set_value(prev_v[:, k].value())

    problem.subject_to(
        atan2(v0_wrt_shooter[2, 0], hypot(v0_wrt_shooter[0, 0], (v0_wrt_shooter[1, 0])))
        == pitch
    )
    problem.minimize(T)
    results = compute_results(problem.solve(), v0_wrt_shooter, T, distance, X)
    if not results:
        print(f"Invalid Pitch: {np.rad2deg(pitch)}")
    return results


def min_velocity(distance: float, target_height: float):
    """
    Solve for minimum velocity.
    :returns: A tuple of [True, velocity, pitch, yaw, X] if it succeeds at a solve, and a tuple of[False, 0] if it fails.
    """
    problem, shooter_wrt_field, target_wrt_field, v0_wrt_shooter, T, X = setup_problem(distance, target_height)

    p_x = X[0, :]
    p_y = X[1, :]
    p_z = X[2, :]

    v = X[3:, :]

    # Position initial guess is linear interpolation between start and end position
    for k in range(N):
        p_x[k].set_value(lerp(shooter_wrt_field[0, 0], target_wrt_field[0, 0], k / N))
        p_y[k].set_value(lerp(shooter_wrt_field[1, 0], target_wrt_field[1, 0], k / N))
        p_z[k].set_value(lerp(shooter_wrt_field[2, 0], target_wrt_field[2, 0], k / N))

    # Velocity initial guess is max initial velocity toward target
    uvec_shooter_to_target = target_wrt_field[:3, :] - shooter_wrt_field[:3, :]
    uvec_shooter_to_target /= norm(uvec_shooter_to_target)
    for k in range(N):
        v[:, k].set_value(
            shooter_wrt_field[3:, :] + max_shooter_velocity * uvec_shooter_to_target
        )

    # Require initial velocity is less than max shooter velocity
    #
    #   √(v_x² + v_y² + v_z²) ≤ v
    #   v_x² + v_y² + v_z² ≤ v²
    #   vᵀv ≤ v²
    initial_speed_squared = v0_wrt_shooter.T @ v0_wrt_shooter
    problem.subject_to(initial_speed_squared <= max_shooter_velocity**2)
    # Minimize initial velocity
    problem.minimize(initial_speed_squared)
    return compute_results(problem.solve(), v0_wrt_shooter, T, distance, X)


def max_velocity(distance: float, target_height: float, min_vel_solve: ShooterSolution):
    # Three stage solve: solve for the average of 90 degrees and the min vel solve's pitch,
    # then solve for 89 degrees pitch, then do the actual max vel solve.
    # The solver likes the fixed pitch solve more than it likes the max vel solve,
    # so use the fixed pitch solve to give the max vel solve a better initial guess.
    # However going straight to 89 degrees poses issues with infeasibility at far ranges,
    # so an intermediate step is introduced.

    # Solves average pitch case
    avg_pitch_solve = fixed_pitch(
        distance,
        target_height,
        (min_vel_solve.pitch + np.deg2rad(90)) / 2,
        min_vel_solve[4],
    )
    if not avg_pitch_solve:
        raise Exception("Fixed pitch solve stage 1 failed")

    # Solves fixed pitch case
    fixed_pitch_solve = fixed_pitch(distance, target_height, np.deg2rad(89), avg_pitch_solve.X)
    if not fixed_pitch_solve:
        raise Exception("Fixed pitch solve stage 2 failed")

    problem, shooter_wrt_field, target_wrt_field, v0_wrt_shooter, T, X = setup_problem(distance, target_height)

    fixed_pitch_p_x = fixed_pitch_solve.X[0, :]
    fixed_pitch_p_y = fixed_pitch_solve.X[1, :]
    fixed_pitch_p_z = fixed_pitch_solve.X[2, :]
    fixed_pitch_v = fixed_pitch_solve.X[3:, :]

    p_x = X[0, :]
    p_y = X[1, :]
    p_z = X[2, :]

    v = X[3:, :]
    v_x = X[3, :]
    v_y = X[4, :]
    v_z = X[5, :]

    # Position initial guess is the fixed pitch solve's position
    for k in range(N):
        p_x[k].set_value(fixed_pitch_p_x[k].value())
        p_y[k].set_value(fixed_pitch_p_y[k].value())
        p_z[k].set_value(fixed_pitch_p_z[k].value())

    # Velocity initial guess is the fixed pitch solve's velocity
    for k in range(N):
        v[:, k].set_value(fixed_pitch_v[:, k].value())

    # Require initial velocity is equal to max shooter velocity
    problem.subject_to(
        (v_x[0] - shooter_wrt_field[3, 0]) ** 2
        + (v_y[0] - shooter_wrt_field[4, 0]) ** 2
        + (v_z[0] - shooter_wrt_field[5, 0]) ** 2
        == max_shooter_velocity**2
    )
    problem.minimize(T)
    return compute_results(problem.solve(), v0_wrt_shooter, T, distance, X)
