// Why do we use kotlin here? Well, kotlin has this feature called operator overloading,
// where you can call operators like +, *, >, >=, etc on non-numbers. This makes
// doing math stuff about 10x clearer when it's done in kotlin vs. java.
package robot.subsystems.shooter

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N6
import lib.Convert
import org.littletonrobotics.junction.Logger
import robot.constants.FieldConstants
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.roundToInt

private const val RHO = 1.221 // Fluid density of air; kg / m^3
private const val C_D = 0.47 // Drag Coefficient
private const val C_L = 0.00025 // Lift Coefficient
private const val BALL_DIAMETER_M = 5.91 * Convert.INCHES_TO_METERS
private const val BALL_MASS = 0.5 * Convert.POUNDS_TO_KG
private val GRAVITATIONAL_ACCEL = VecBuilder.fill(0.0, 0.0, -9.81)

/**
 * Simulates a shot with an initial shooter position + ball velocity.
 * Logs the simulated shot as an array of Pose3ds, while returning the total
 * time of the trajectory in seconds.
 */
fun simulateShot(pose: Translation3d, velocity: Translation3d, dt: Double): Double {
    // For a ball with full backspin, the direction of the rotational velocity vector is clockwise
    // 90 degrees from the direction of travel and the magnitude is the launch velocity / diameter
    val omegaMag = velocity.norm / BALL_DIAMETER_M
    val rotatedYaw = velocity.toTranslation2d().angle.plus(Rotation2d.kCW_Pi_2)
    val omega = VecBuilder.fill(omegaMag * rotatedYaw.cos, omegaMag * rotatedYaw.sin, 0.0)
    var x = VecBuilder.fill(
        pose.x, pose.y, pose.z,
        velocity.x, velocity.y, velocity.z
    )
    val poses = mutableListOf<Pose3d>()
    val blueHub = FieldConstants.Hub.topCenterPoint
    val redHub = FieldConstants.Hub.oppTopCenterPoint
    var t = 0.0
    while (t < 10 && !(x[2] < 0 || ballIsIn(blueHub, x) || ballIsIn(redHub, x))) {
        // if 0.05 seconds have passed, add a ball position to the list of positions.
        if ((t * 100.0).roundToInt() % 5 == 0) poses.add(Pose3d(x[0], x[1], x[2], Rotation3d.kZero))
        x = computeNextX(x, omega, dt)
        t += dt
    }
    poses.add(Pose3d(x[0], x[1], x[2], Rotation3d.kZero))
    Logger.recordOutput("ShotCalcs/Simulated Shot", *poses.toTypedArray())
    return t
}

/**
 * Computes the next state of the ball given the previous state (x),
 * the angular velocity omega, and a delta time dt.
 * We use RK4 integration, which is a more accurate version of euler's method.
 * An example of euler's method would be:
 * ```return x + equationOfMotion(x, omega) * dt```
 */
private fun computeNextX(x: Vector<N6>, omega: Vector<N3>, dt: Double): Vector<N6> {
    val k1 = equationOfMotion(x, omega)
    val k2 = equationOfMotion(x + k1 * dt / 2, omega)
    val k3 = equationOfMotion(x + k2 * dt / 2, omega)
    val k4 = equationOfMotion(x + k3 * dt, omega)
    val dx = (k1 + k2 * 2.0 + k3 * 2.0 + k4) * dt / 6.0
    return x + dx
}

/**
 * Returns an array of [velocity x, velocity y, velocity z, accel x, accel y, accel z],
 * given x = [x, y, z, velocity x, velocity y, velocity z]
 * and omega = [roll, pitch, yaw].
 */
private fun equationOfMotion(x: Vector<N6>, omega: Vector<N3>): Vector<N6> {
    // x" = −F_D(v) / mass * v̂_x
    // y" = −F_D(v) / mass * v̂_y
    // z" = −g − F_D(v) / mass * v̂_z
    //
    // Per https://en.wikipedia.org/wiki/Drag_(physics)#The_drag_equation:
    // F_D(v) = ½ρv²C_D A
    // ρ(rho) is the fluid density in kg/m³
    // v is the velocity magnitude in m/s
    // C_D is the drag coefficient (dimensionless)
    // A is the cross-sectional area of a circle in m²
    // v̂ is the velocity direction unit vector
    val v = VecBuilder.fill(x[3], x[4], x[5])
    val r = BALL_DIAMETER_M / 2
    val A = Math.PI * r.pow(2)
    val dragForce = 0.5 * RHO * v.norm().pow(2) * C_D * A

    val v_hat = v / v.norm()
    val liftForce = Vector.cross(v, omega) * 0.5 * RHO * C_L * A * v.norm()
    val a = v_hat * (-dragForce / BALL_MASS) + (liftForce / BALL_MASS) + GRAVITATIONAL_ACCEL
    return VecBuilder.fill(v[0], v[1], v[2], a[0], a[1], a[2])
}

/** Checks if the ball is within the designated scoring location. */
private fun ballIsIn(scoreLocation: Translation3d, ballState: Vector<N6>): Boolean {
    val inHubZone = hypot(ballState[0] - scoreLocation.x, ballState[1] - scoreLocation.y) < 47.0 / 2 * Convert.INCHES_TO_METERS
    return inHubZone && ballState[2] - scoreLocation.z < 0
}