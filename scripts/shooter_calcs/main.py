"""
Copied from 167 - thanks lol

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
import numpy as np

from shooter_calcs.internals import min_velocity, max_velocity, lerp, fixed_pitch

delta_pitch = np.deg2rad(2.5)

def iterate_distance(file, distance, target_height):
    # Solve for minimum velocity
    min_vel_solve = min_velocity(distance, target_height)
    if not min_vel_solve:
        return None

    # Solve for maximum velocity
    max_vel_solve = max_velocity(distance, target_height, min_vel_solve)
    if not max_vel_solve:
        raise Exception("Max vel solve failed")

    # If the position is possible, lerp between min velocity and max velocity
    # to search the in between velocities
    pitch_samples = math.ceil((max_vel_solve.pitch_rad - min_vel_solve.pitch_rad) / delta_pitch)

    file.write("    put(\n")
    file.write(f"        {distance},\n")
    file.write(f"        entry({min_vel_solve.velocity_mps}, new ShotResult({min_vel_solve.pitch_rad},"
               f" {min_vel_solve.time_secs})),\n")
    prev_solve = min_vel_solve
    for i in range(1, pitch_samples - 1):
        pitch = lerp(min_vel_solve.pitch, max_vel_solve.pitch, i / (pitch_samples - 1))
        solve = fixed_pitch(distance, target_height, pitch, prev_solve.X)
        if solve:
            file.write(f"        entry({solve.velocity_mps}, new ShotResult({solve.pitch_rad}, {solve.time_secs})),\n")
            prev_solve = solve
        else:
            break
        if pitch + delta_pitch > max_vel_solve.pitch_rad:
            break
    file.write(f"        entry({max_vel_solve.velocity_mps}, new ShotResult({max_vel_solve.pitch_rad}, "
               f"{max_vel_solve.time_secs})));\n")
    return min_vel_solve.pitch_rad


# def write(file, target_height, name):


if __name__ == "__main__":
    pass
    # with open("../../src/main/java/robot/subsystems/shooter/math/ShooterMathConsts.java") as file:
    #     file.write("package robot.subsystems.shooter.math;\n\n")
    #
    #     file.write("import static java.util.Map.entry;\n\n")
    #
    #     file.write(f"public class {name} " "{\n")
    #     file.write(f"  public {name}() " "{\n")
    #
    #     for i in range(distance_samples):
    #         distance = lerp(
    #             start_distance,
    #             end_distance,
    #             (i / (distance_samples - 1)) ** distance_exponent,
    #             )
    #         iterate_distance(file, distance, target_height)
    #
    #     file.write("  }\n")
    #     file.write("}")
    #     file.close()
    # write(open("../src/main/java/frc/cotc/shooter/HubShotMap.java", "w"), 72 * .0254, "HubShotMap")
    # write(open("../src/main/java/frc/cotc/shooter/GroundShotMap.java", "w"), 0, "GroundShotMap")