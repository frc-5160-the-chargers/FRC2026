// We use kotlin to write our shooter calculations code because it has operator overloading (w)
package robot.subsystems.shooter.math

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N6
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import lib.Convert
import robot.subsystems.shooter.ShooterConsts.*
import kotlin.math.exp
import kotlin.math.pow
import kotlin.math.sqrt


fun computeSetpoint(
    target: Translation2d,
    shotMap: ShotMap,
    robotPose: Pose2d,
    fieldCentricVel: ChassisSpeeds,
    currentShooterVel: AngularVelocity
): ShooterSetpoint {
    val fieldOriginToShooter = (robotPose + ROBOT_TO_SHOOTER).translation
    var shooterToTarget = target - fieldOriginToShooter

    val omega = fieldCentricVel.omegaRadiansPerSecond
    val shooterVx = fieldCentricVel.vxMetersPerSecond + omega * ROBOT_TO_SHOOTER.x
    val shooterVy = fieldCentricVel.vyMetersPerSecond - omega * ROBOT_TO_SHOOTER.y
    val nextDesiredVel = ANGULAR_TO_LINEAR_VEL.get(currentShooterVel.`in`(RadiansPerSecond))
    val finalDesiredVel = shotMap.maxVelocityAt(shooterToTarget.norm)
    val shotValid = shotMap.canShoot(shooterToTarget.norm, nextDesiredVel)

    lateinit var solve: ShotMapResult
    repeat(10) {
        solve = shotMap.get(shooterToTarget.norm, nextDesiredVel)
        val dragComp = DRAG_COMPENSATION.get()
        val timeCompensation = LOOKAHEAD_SECS.get() +
            (1 - exp(-solve.airTimeSecs * dragComp)) / dragComp
        shooterToTarget += Translation2d(shooterVx * timeCompensation, shooterVy * timeCompensation)
    }

    return ShooterSetpoint(
        shotValid, shooterToTarget.angle, Rotation2d.fromRadians(solve.pitchRad),
        RadiansPerSecond.of(ANGULAR_TO_LINEAR_VEL.getKey(finalDesiredVel))
    )
}

fun simulateShot(): Array<Pose3d> {
    return arrayOf()
}


private const val RHO = 1.204
private const val C_D = 0.4 // Drag Coefficient
private const val C_L = 0.00025 // Lift Coefficient
private const val BALL_DIAMETER_M = 5.91 * Convert.INCHES_TO_METERS
private const val BALL_MASS = 0.5 * Convert.POUNDS_TO_KG
private val GRAVITATIONAL_ACCEL = VecBuilder.fill(0.0, 0.0, -9.81)


/**
 * Returns an array of [velocity x, velocity y, velocity z, accel x, accel y, accel z],
 * given x = [x, y, z, velocity x, velocity y, velocity z]
 * and omega = [roll, pitch, yaw].
 */
private fun equationOfMotion(x: Vector<N6>, omega: Vector<N3>): Vector<N6> {
    // x' = x'
    // y' = y'
    // z' = z'
    // x" = −F_D(v)/m v̂_x
    // y" = −F_D(v)/m v̂_y
    // z" = −g − F_D(v)/m v̂_z
    //
    // Per https://en.wikipedia.org/wiki/Drag_(physics)#The_drag_equation:
    //   F_D(v) = ½ρv²C_D A
    //   ρ is the fluid density in kg/m³
    //   v is the velocity magnitude in m/s
    //   C_D is the drag coefficient (dimensionless)
    //   A is the cross-sectional area of a circle in m²
    //   m is the mass in kg
    //   v̂ is the velocity direction unit vector
    val v = VecBuilder.fill(x[3], x[4], x[5])
    val v2 = v[0].pow(2) + v[1].pow(2) + v[2].pow(2)
    val v_mag = sqrt(v2)
    val r = BALL_DIAMETER_M / 2
    val A = Math.PI * r.pow(2)
    val dragForce = 0.5 * RHO * v2 * C_D * A

    val v_hat = v / v_mag
    val liftForce = Vector.cross(v, omega) * 0.5 * RHO * C_L * A * v_mag
    val a = v_hat * (-dragForce / BALL_MASS) + (liftForce / BALL_MASS) + GRAVITATIONAL_ACCEL
    return VecBuilder.fill(v[0], v[1], v[2], a[0], a[1], a[2])
}