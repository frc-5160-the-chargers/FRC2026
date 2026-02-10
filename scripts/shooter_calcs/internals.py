import math

import numpy as np

from numpy.linalg import norm
from sleipnir import autodiff
from sleipnir.autodiff import VariableMatrix, sqrt
from sleipnir.optimization import ExitStatus, Problem

from shooter_calcs.data_types import ShooterSolution

# Physical characteristics
shooter_height = 20 * 0.0254  # m
g = 9.81  # m/s²
max_shooter_velocity = 14.5  # m/s
ball_mass = 0.5 / 2.205  # kg
ball_diameter = 5.91 * 0.0254  # m
rho = 1.204  # Density of air, kg/m³
wheel_radius = 2 * 0.0254 # m
C_D = 0.4 # Coefficient of drag, unitless
C_L = 0.5 # Coefficient of lift, unitless
min_pitch = np.deg2rad(40)

# Solve settings
N = 40 # The number of iterations per solve


def lerp(a, b, t):
    return a + t * (b - a)


def f(x):
    """
    Calculates a vector containing 3-axis velocity and accel
    in the form (vx, vy, vz, ax, ay, az), given a state of
    (x, y, z, vx, vy, vz).
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
    A = math.pi * ((ball_diameter / 2) ** 2)
    drag_force = lambda v: 0.5 * rho * v**2 * C_D * A
    a_D = lambda v: drag_force(v) / ball_mass

    v_x = x[3, 0]
    v_y = x[4, 0]
    v_z = x[5, 0]

    # Magnus force: F_L(v) = ½ρv² C_L A |ω x v|
    lift_force = lambda v: 0.5 * rho * magnitude_squared(v) * C_L * A * np.cross(np.array([0, v[1]/wheel_radius, 0]), v)
    a_L = lift_force(np.array([v_x, v_y, v_z])) / ball_mass

    return VariableMatrix(
        [[v_x], [v_y], [v_z], [-a_D(v_x) - a_L[0]], [-a_D(v_y) - a_L[1]], [-g - a_D(v_z) - a_L[2]]]
    )


def find_pitch(v0_wrt_shooter):
    return autodiff.atan2(
        v0_wrt_shooter[2, 0],
        autodiff.hypot(v0_wrt_shooter[0, 0], (v0_wrt_shooter[1, 0]))
    )


def magnitude_squared(vector):
    #   √(v_x² + v_y² + v_z²) ≤ v
    #   v_x² + v_y² + v_z² ≤ v²
    #   vᵀv ≤ v²
    return vector.T @ vector


def setup_problem(distance: float, target_height: float):
    """
    Set up the problem and any shared constraints between the two solve modes (min and fix vel)
    """
    # Robot initial state. Order: (x, y, z, vx, vy, vz)
    shooter_wrt_field = np.array([[0], [0], [shooter_height], [0.0], [0.0], [0.0]])
    target_wrt_field = np.array([[distance], [0], [target_height], [0.0], [0.0], [0.0]])

    problem = Problem()

    # Set up duration decision variables
    T = problem.decision_variable()
    problem.subject_to(T >= 0)
    T.set_value(1)
    dt = T / N

    # Ball state in field frame. Order: (x, y, z, vx, vy, vz)
    X = problem.decision_variable(6, N)

    p = X[:3, :]
    v_x = X[3, :]
    v_y = X[4, :]
    v_z = X[5, :]

    # The initial velocity of the ball from the shooter perspective.
    v0_wrt_shooter = X[3:, :1] - shooter_wrt_field[3:, :]

    # Dynamics constraints - RK4 integration
    # See https://www.youtube.com/watch?v=dShtlMl69kY
    h = dt
    for k in range(N - 1):
        x_k = X[:, k]
        x_k1 = X[:, k + 1]

        k1 = f(x_k)
        k2 = f(x_k + h / 2 * k1)
        k3 = f(x_k + h / 2 * k2)
        k4 = f(x_k + h * k3)
        problem.subject_to(x_k1 == x_k + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4))
    # Initial position must be shooter initial position
    problem.subject_to(p[:, :1] == shooter_wrt_field[:3, :])
    # Final position must be in center of target circle
    problem.subject_to(p[:, -1] == target_wrt_field[:3, :])
    # Require the final velocity is at least somewhat downwards by limiting horizontal velocity
    # and requiring negative vertical velocity
    problem.subject_to(v_z[-1] < -1)
    # Max horizontal velocity is 2.5 times the downwards velocity (~21 degrees from horizontal)
    problem.subject_to(autodiff.hypot(v_x[-1], v_y[-1]) <= v_z[-1] * -5)
    # Pitch must be higher than min pitch
    problem.subject_to(find_pitch(v0_wrt_shooter) >= min_pitch)

    return problem, shooter_wrt_field, target_wrt_field, v0_wrt_shooter, T, X


def provide_initial_guess(target: VariableMatrix, initial_guess: VariableMatrix):
    """
    Provides the target state with an "initial guess point" by copying data
    from the initial_guess state. Each state must have the format (x, y, z, vx, vy, vz).
    """
    prev_p_x = initial_guess[0, :]
    prev_p_y = initial_guess[1, :]
    prev_p_z = initial_guess[2, :]
    prev_v = initial_guess[3:, :]

    p_x = target[0, :]
    p_y = target[1, :]
    p_z = target[2, :]
    v = target[3:, :]

    # Position initial guess is last solve's position
    for k in range(N):
        p_x[k].set_value(prev_p_x[k].value())
        p_y[k].set_value(prev_p_y[k].value())
        p_z[k].set_value(prev_p_z[k].value())

    # Velocity initial guess is last solve's velocity
    for k in range(N):
        v[:, k].set_value(prev_v[:, k].value())


def compute_results(status, v0_wrt_shooter, T, distance, X):
    if status == ExitStatus.SUCCESS:
        # Initial velocity vector with respect to shooter
        v0 = v0_wrt_shooter.value()
        velocity = norm(v0)
        pitch = math.atan2(v0[2, 0], math.hypot(v0[0, 0], v0[1, 0]))
        time = T.value()
        return ShooterSolution(velocity_mps=velocity, pitch_rad=pitch, time_secs=time, X=X)
    print(f"Infeasible at distance {distance:.03f} m with status {status.name}")
    return None


def min_velocity(distance: float, target_height: float):
    """
    Calculates a ShooterSolution that minimizes the initial velocity of the ball,
    given a distance and target height.
    """
    problem, shooter_wrt_field, target_wrt_field, v0_wrt_shooter, T, X = setup_problem(
        distance, target_height
    )

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
    speed_squared = magnitude_squared(v0_wrt_shooter)
    problem.subject_to(speed_squared <= max_shooter_velocity**2)
    # Minimize initial velocity
    problem.minimize(speed_squared)
    # Return results
    return compute_results(problem.solve(), v0_wrt_shooter, T, distance, X)


def solve_fixed_pitch(
    distance: float,
    target_height: float,
    pitch: float,
    starting_point: ShooterSolution
):
    """
    Calculates a ShooterSolution that minimizes the airtime of the ball with a fixed pitch.
    This needs a starting point to initialize its values,
    which ideally comes from solve_min_velocity() or a previous solve_fixed_pitch() call.
    """
    problem, shooter_wrt_field, target_wrt_field, v0_wrt_shooter, T, X = \
        setup_problem(distance, target_height)
    provide_initial_guess(target=X, initial_guess=starting_point.X)

    problem.subject_to(find_pitch(v0_wrt_shooter) == pitch)
    problem.minimize(T)
    return compute_results(problem.solve(), v0_wrt_shooter, T, distance, X)


def max_velocity(distance: float, target_height: float, starting_point: ShooterSolution):
    """
    Calculates a ShooterSolution that minimizes the airtime of the ball while maximizing
    the initial velocity of said ball.

    This needs a starting point to initialize its values,
    which ideally comes from a solve_min_velocity() call.

    Three stage solve: solve for the average of 90 degrees and the min vel solve's pitch,
    then the average of that angle and 90 degrees, then do the max vel solve
    The solver likes the fixed pitch solve more than it likes the max vel solve,
    so use the fixed pitch solve to give the max vel solve a better initial guess.
    """
    stage_1_solve = solve_fixed_pitch(
        distance,
        target_height,
        (starting_point.pitch_rad + np.deg2rad(90)) / 2,
        starting_point,
    )
    if not stage_1_solve:
        raise Exception("Fixed pitch solve stage 1 failed")
    stage_2_solve = solve_fixed_pitch(
        distance,
        target_height,
        (stage_1_solve.pitch_rad + np.deg2rad(90)) / 2,
        stage_1_solve,
    )
    if not stage_2_solve:
        raise Exception("Fixed pitch solve stage 2 failed")

    problem, shooter_wrt_field, target_wrt_field, v0_wrt_shooter, T, X = \
        setup_problem(distance, target_height)
    provide_initial_guess(target=X, initial_guess=stage_2_solve.X)

    problem.subject_to(magnitude_squared(v0_wrt_shooter) == max_shooter_velocity**2)
    problem.minimize(T)
    return compute_results(problem.solve(), v0_wrt_shooter, T, distance, X)
