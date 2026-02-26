"""
FRC 2026 shooter trajectory optimization.

This program uses the Sleipnir NLP solver to find the initial pitch and yaw for a game
piece to hit the 2026 FRC game's target given an initial velocity.

This optimization problem formulation uses direct transcription of the flight dynamics, including
air resistance.

Based on the 2022 trajectory optimization example code by Tyler Veness.
https://github.com/SleipnirGroup/Sleipnir/blob/main/examples/frc_2022_shooter/main.py

Required packages:
- numpy
- sleipnirgroup-jormungandr
"""

import math
from math import ceil

import numpy as np
from numpy.linalg import norm
from sleipnir.autodiff import VariableMatrix, atan2, block, cos, hypot, sin, sqrt
from sleipnir.optimization import ExitStatus, Problem

from shooter_calcs.data_types import ShooterSolution, SolveMode, MinVelocityWithPitch

# Physical characteristics
rho = 1.221  # kg/m³
g = np.array([[0], [0], [-9.81]])  # m/s²
ball_mass = 0.5 / 2.205  # kg
ball_diameter = 5.91 * 0.0254  # m

C_D = 0.47 # Drag Coefficient; dimensionless
C_L = 0.00025 # Lift Coefficient; dimensionless
shooter_height = 14.9 * 0.0254  # m
min_pitch = np.deg2rad(45)  # rad
max_pitch = np.deg2rad(85)  # rad
max_shooter_speed = 14.5  # m/s

# Solve Settings
print_results = False
N = 40 # The number of steps the solver takes to arrive to a solution.
delta_pitch = np.deg2rad(2.5) # The pitch difference between every attempted solve.


def lerp(a, b, t):
    return a + t * (b - a)


def cross(u, v):
    return VariableMatrix(
        [
            [u[1, 0] * v[2, 0] - u[2, 0] * v[1, 0]],
            [-u[0, 0] * v[2, 0] + u[2, 0] * v[0, 0]],
            [u[0, 0] * v[1, 0] - u[1, 0] * v[0, 0]],
        ]
    )


def magnitude_squared(vector):
    #   √(u_0² + u_1² + u_2²) = v
    #   u_0² + u_1² + u_2² = v²
    #   uᵀu = v²
    return vector.T @ vector


def equation_of_motion(x, omega):
    # Derives an output of [vx, vy, vz, ax, ay, az],
    # Given inputs x=[x, y, z, vx, vy, vz] and omega=[roll, pitch, yaw].
    # Per https://en.wikipedia.org/wiki/Drag_(physics)#The_drag_equation:
    #   F_D(v) = ½ρv²C_D A
    #   ρ is the fluid density in kg/m³
    #   v is the velocity magnitude in m/s
    #   C_D is the drag coefficient (dimensionless)
    #   A is the cross-sectional area of a circle in m²
    #   m is the mass in kg
    #   v̂ is the velocity direction unit vector
    v = x[3:6, :]  # m/s
    v2 = magnitude_squared(v)[0, 0]
    v_mag = sqrt(v2)
    r = ball_diameter / 2
    A = math.pi * r**2  # m²
    m = ball_mass
    F_D = 0.5 * rho * v2 * C_D * A

    # Magnus force formula
    #   F_M(v) = ½ρvAC_L (v̂ × ω)
    #   C_L is the lift coefficient (dimensionless)
    v_hat = v / v_mag
    F_M = 0.5 * rho * C_L * A * v_mag * cross(v, omega)

    return block([[v], [g - F_D / m * v_hat + F_M / m]])


def solve(
    mode: SolveMode,
    distance: float,
    target_height: float,
    last_solve: ShooterSolution | None = None,
    trying_again: bool = False,
) -> ShooterSolution | None:
    """
    Uses sleipnir to calculate a ShooterSolution.
    :param mode: Can solve for min_velocity, max_velocity, or a minimum velocity
    with a pitch constraint.
    :param distance: The distance between the shooter and the target.
    :param target_height: The height of the goal.
    :param last_solve: The sleipnir solver can be pretty finicky at times;
    so, we occasionally use a previous solution as a starting point for solves.
    :param trying_again: Whether we are retrying the solve (see line 242).

    :returns: A ShooterSolution if the solve is successful, and None otherwise.
    """
    # Robot initial state (format: [x,y,z,roll,pitch,yaw])
    shooter_wrt_field = np.array([[0], [0], [shooter_height], [0], [0], [0]])
    target_wrt_field = np.array([[distance], [0], [target_height], [0], [0], [0]])

    problem = Problem()

    # Set up duration decision variables
    T = problem.decision_variable() # The air time of the ball.
    problem.subject_to(T >= 0)
    T.set_value(last_solve.time_secs if last_solve else 1)
    dt = T / N

    # Ball state in field frame.
    # X = [x, y, z, vx, vy, vz]
    X: VariableMatrix = problem.decision_variable(6, N)
    p = X[:3, :] # The position of the ball at every moment, as [x,y,z].
    v = X[3:, :] # The velocity of the ball at every moment, as [vx, vy, vz].
    v_z = X[5, :] # The z velocity of the ball at every moment.
    # A vector representing the initial velocity of the ball.
    v0_wrt_shooter = X[3:, :1] - shooter_wrt_field[3:, :]

    # The ball's position(p) must start at the shooter's position(shooter_wrt_field).
    problem.subject_to(p[:, :1] == shooter_wrt_field[:3, :])

    # The initial angular velocity of the ball as [roll, pitch, yaw]. Don't ask me why it's a decision variable.
    omega = problem.decision_variable(3, 1)
    omega_magnitude = magnitude_squared(v0_wrt_shooter) / ball_diameter
    omega_direction = atan2(v0_wrt_shooter[1, 0], v0_wrt_shooter[0, 0]) - math.pi / 2
    problem.subject_to(omega[0, 0] == omega_magnitude * cos(omega_direction))
    problem.subject_to(omega[1, 0] == omega_magnitude * sin(omega_direction))
    problem.subject_to(omega[2, 0] == 0)
    if last_solve is None:
        omega[0, 0].set_value(-max_shooter_speed / ball_diameter)
    else:
        omega[0, 0].set_value(-last_solve.velocity_mps / ball_diameter)

    # Uses integration to constrain the position of the ball to the ball's actual pathway of motion.
    # The simplest way would be to use euler's method:
    # problem.subject_to(x_k1 == x_k + equation_of_motion(x_k, omega) * dt)
    # Here, we instead use something called RK4 integration, which is essentially
    # a more accurate version of euler's method that takes the average of many iterations.
    h = dt
    for k in range(N - 1):
        x_k = X[:, k]
        x_k1 = X[:, k + 1]
        k1 = equation_of_motion(x_k, omega)
        k2 = equation_of_motion(x_k + h / 2 * k1, omega)
        k3 = equation_of_motion(x_k + h / 2 * k2, omega)
        k4 = equation_of_motion(x_k + h * k3, omega)
        problem.subject_to(x_k1 == x_k + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4))

    # Require final position is in center of target circle
    problem.subject_to(p[:, -1] == target_wrt_field[:3, :])

    # Require the final velocity is at least somewhat downwards by
    # requiring negative vertical velocity at the end of the ball's path.
    problem.subject_to(v_z[-1] < -1)

    if last_solve is None:
        p_x = X[0, :]
        p_y = X[1, :]
        p_z = X[2, :]
        # Position initial guess is linear interpolation between start and end position
        for k in range(N):
            p_x[k].set_value(
                lerp(shooter_wrt_field[0, 0], target_wrt_field[0, 0], k / N)
            )
            p_y[k].set_value(
                lerp(shooter_wrt_field[1, 0], target_wrt_field[1, 0], k / N)
            )
            p_z[k].set_value(
                lerp(shooter_wrt_field[2, 0], target_wrt_field[2, 0], k / N)
            )

        # Velocity initial guess is max initial velocity toward target
        uvec_shooter_to_target = target_wrt_field[:3, :] - shooter_wrt_field[:3, :]
        uvec_shooter_to_target /= norm(uvec_shooter_to_target)
        for k in range(N):
            v[:, k].set_value(
                shooter_wrt_field[3:, :] + max_shooter_speed * uvec_shooter_to_target
            )
    else:
        X.set_value(last_solve.X)

    initial_shot_speed = magnitude_squared(v0_wrt_shooter)
    # Ensure that the hood angle is between min and max pitch.
    pitch = atan2(v0_wrt_shooter[2, 0], hypot(v0_wrt_shooter[0, 0], v0_wrt_shooter[1, 0]))
    problem.subject_to(pitch <= max_pitch)
    problem.subject_to(pitch >= min_pitch)
    # Require that this solve's pitch exceeds the last solve's pitch.
    if last_solve is not None:
        problem.subject_to(pitch > last_solve.pitch_rad)

    match mode:
        case "min_velocity":
            # Require initial velocity is less than max shooter velocity
            problem.subject_to(initial_shot_speed <= max_shooter_speed ** 2)
            # Minimize initial velocity
            problem.minimize(initial_shot_speed)
        case "max_velocity":
            # Maximize initial velocity
            problem.maximize(initial_shot_speed)
            problem.solve()
            # Require initial velocity is less than max shooter velocity
            problem.subject_to(initial_shot_speed <= max_shooter_speed ** 2)
        case MinVelocityWithPitch(pitch_rad=target_pitch):
            problem.subject_to(pitch == target_pitch)
            # Minimize initial velocity
            problem.minimize(initial_shot_speed)
        case _:
            raise Exception("Invalid Mode Given.")

    status = problem.solve()
    if status == ExitStatus.SUCCESS:
        velocity = norm(v0_wrt_shooter.value())
        pitch = pitch.value()
        time = T.value()
        if print_results:
            print(f"Mode {mode} solve at distance {distance:.03f} m")
            print(f"Velocity = {velocity:.03f} m/s")
            print(f"Pitch = {np.rad2deg(pitch):.03f}°")
            print(f"Time = {time:.03f} s")
        return ShooterSolution(velocity_mps=velocity, pitch_rad=pitch, time_secs=time, X=X.value())
    if not trying_again and not isinstance(mode, MinVelocityWithPitch):
        # Sometimes, using an anchor solve (via last_solve) will actually
        # negatively impact the solver's ability to converge.
        # As a result, for non fixed-pitch cases, we occasionally try again without the anchor point.
        # Why? Don't ask me lmao (a lot of this code is copied)
        if print_results:
            print(f"Solve failed with status {status.name}, trying again")
        return solve(mode, distance, target_height, trying_again=True)
    if isinstance(mode, MinVelocityWithPitch):
        print(
            f"Mode 2 solve failed at distance {distance:.03f} m and pitch "
            f"{np.rad2deg(mode.pitch_rad):.03f}°"
        )
    else:
        print(f"Mode {mode} solve failed at distance {distance:.03f} m")
    return None


def write_distance_entry(file, distance, target_height, last_min_vel_solve):
    """
    Writes velocity to air time and velocity to hood angle mappings
    for a specific distance from the target.
    """
    min_vel_solve = solve("min_velocity", distance, target_height, last_min_vel_solve)
    if not min_vel_solve:
        return None
    file.write("\n")
    file.write(f"        put(\n")
    file.write(f"            {distance:.05f},\n")
    write_velocity_entries(file, distance, target_height, min_vel_solve)
    file.write("        );")
    return min_vel_solve


def write_velocity_entries(file, distance, target_height, min_vel_solve):
    """
    Implementation detail behind write_distance_entry.
    """
    solves = [min_vel_solve]
    max_vel_solve = solve("max_velocity", distance, target_height, min_vel_solve)
    # If we can solve for a ShooterSolution with max velocity, set the upper bound pitch
    # to that solve's pitch. Otherwise, set the upper bound to the max pitch the hood can hit.
    # Then, solve for ShooterSolution(s) for a series of pitch values in between the pitch
    # from shooting the ball with the least velocity, and the upper bound pitch.
    if max_vel_solve:
        samples = ceil((max_vel_solve.pitch_rad - min_vel_solve.pitch_rad) / delta_pitch)
        last_solve = min_vel_solve
        for i in range(1, samples):
            target_pitch = lerp(min_vel_solve.pitch_rad, max_vel_solve.pitch_rad, i / samples)
            fixed_pitch_solve = solve(
                MinVelocityWithPitch(pitch_rad=target_pitch),
                distance,
                target_height,
                last_solve,
            )
            if fixed_pitch_solve:
                solves.append(fixed_pitch_solve)
                last_solve = fixed_pitch_solve
        solves.append(max_vel_solve)
    else:
        pitch = min_vel_solve.pitch_rad + delta_pitch
        last_solve = min_vel_solve
        while pitch <= max_pitch:
            fixed_pitch_solve = solve(
                MinVelocityWithPitch(pitch_rad=pitch),
                distance,
                target_height,
                last_solve,
            )
            if fixed_pitch_solve:
                solves.append(fixed_pitch_solve)
                last_solve = fixed_pitch_solve
            pitch += delta_pitch
        max_vel_solve = solve("max_velocity", distance, target_height, last_solve)
        if max_vel_solve:
            solves.append(max_vel_solve)

    for i in range(len(solves)):
        line_ending = ",\n" if i < len(solves) - 1 else "\n"
        file.write(f"            {solves[i].format()}{line_ending}")


def write(
    target_height: float,
    min_distance: float,
    max_distance: float,
    base_delta_distance: float,
    name: str,
):
    """
    Writes a file containing shot paramters for a specific location on the field.
    In 2026, this is either the hub or a random place on the ground (for ferrying purposes).
    """
    file = open(f"../../src/main/java/robot/subsystems/shooter/math/{name}.java", "w")
    file.write(f"""// This file is autogenerated at 'scripts/shooter_calcs/main.py'.
package robot.subsystems.shooter.math;

import robot.subsystems.shooter.DataTypes.ShotMapResult;

import static java.util.Map.entry;

public final class {name} extends ShotMap {{
    public {name}() {{""")

    last_min_vel_solve = None
    delta_distance = base_delta_distance
    distance = min_distance
    while True:
        distance_solve = write_distance_entry(
            file, distance, target_height, last_min_vel_solve
        )
        if distance_solve:
            last_min_vel_solve = distance_solve
            delta_distance = base_delta_distance
            distance += delta_distance
            if distance > max_distance:
                break
        else:
            # Copied this from 167 so don't ask me why it works
            # Basically though, if we fail a solve, we try smaller and smaller distance values
            # Until it works.
            distance -= delta_distance
            delta_distance /= 2
            if delta_distance < 0.01:
                delta_distance = base_delta_distance / 2
            distance += delta_distance * 1.5
            if distance > max_distance:
                break
    write_distance_entry(file, max_distance, target_height, last_min_vel_solve)
    file.write("\n")

    file.write("    }\n")
    file.write("}\n")
    file.close()
    print(f"Done writing {name}.java")


if __name__ == "__main__":
    write(
        target_height=72 * 0.0254,
        min_distance=0.8,
        max_distance=6,
        base_delta_distance=0.1,
        name="HubShotMap",
    )
    # write(
    #     target_height=0,
    #     min_distance=0.5,
    #     max_distance=15.5,
    #     base_delta_distance=0.5,
    #     name="GroundShotMap"
    # )