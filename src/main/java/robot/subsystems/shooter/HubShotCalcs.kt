// Why do we use kotlin here? Well, kotlin has this feature called operator overloading,
// where you can call operators like +, *, >, >=, etc on non-numbers. This makes
// doing math stuff about 10x clearer when it's done in kotlin vs. java.
@file:Suppress("LocalVariableName", "Unused")
package robot.subsystems.shooter

import choreo.util.ChoreoAllianceFlipUtil.flip
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.*
import lib.AllianceColor
import lib.RobotMode
import lib.Tunable
import lib.TunableLerpTable
import org.littletonrobotics.junction.Logger
import robot.constants.FieldConstants
import kotlin.math.roundToInt

/** Calculates the desired shot parameters to launch fuel into the hub. */
fun calcHubShotSetpoint(robotPose: Pose2d, fieldCentricVel: ChassisSpeeds): Shooter.Setpoint {
    val goalPosition = if (AllianceColor.isRed()) flip(HUB_LOCATION) else HUB_LOCATION
    val iterations = Array(NEWTONS_METHOD_ITERATIONS + 1) { Pose2d.kZero }
    val vx = fieldCentricVel.vxMetersPerSecond
    val vy = fieldCentricVel.vyMetersPerSecond
    val omega = fieldCentricVel.omegaRadiansPerSecond

    // In order to compensate for processing delay, we don't use the robot's pose directly;
    // rather, we use the robot pose a small amount of time(0.05 secs-ish) into the future.
    // We multiply this time by the robot's velocity in the x, y, and omega directions,
    // and then add it to each component of the robot's pose to get the new "future" pose.
    val lookaheadComp = Transform2d(vx, vy, Rotation2d.fromRadians(omega)) * LOOKAHEAD_SECS.get()
    val fieldOriginToShooter = (robotPose + lookaheadComp + Shooter.ROBOT_TO_LAUNCH_POINT).translation
    val shooterToGoal = goalPosition - fieldOriginToShooter
    iterations[0] = Pose2d(fieldOriginToShooter, shooterToGoal.angle)
    Logger.recordOutput("ShotCalcs/InitialDistanceToGoal(m)", shooterToGoal.norm)

    val botToLaunchPointRotated = fieldOriginToShooter - robotPose.translation
    val shooterVelocity = Translation2d(
        vx + omega * botToLaunchPointRotated.x,
        vy - omega * botToLaunchPointRotated.y
    )
    // The time it takes for a ball to reach the goal.
    var airTime = DISTANCE_TO_AIR_TIME.get(shooterToGoal.norm)
    repeat(NEWTONS_METHOD_ITERATIONS) { i ->
        // In this case, futureShooterToGoal is the x and y distance between the shooter and the goal
        // by the time the ball has already landed within the goal.
        val futureFieldOriginToShooter = fieldOriginToShooter + shooterVelocity * airTime
        val futureShooterToGoal = goalPosition - futureFieldOriginToShooter
        val distanceToGoal = futureShooterToGoal.norm
        iterations[i + 1] = Pose2d(futureFieldOriginToShooter, futureShooterToGoal.angle)

        // Newton's method: see https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/newton-shooting.html
        val error = airTime - DISTANCE_TO_AIR_TIME.get(distanceToGoal)
        val dt_dD = 1 / DISTANCE_TO_BALL_SPEED.get(distanceToGoal) // just an estimate
        val dD_dt = -futureShooterToGoal.dot(shooterVelocity) / distanceToGoal
        val dError_dt = 1 - dt_dD * dD_dt
        airTime -= error / dError_dt
    }
    val futureShooterToGoal = shooterToGoal - shooterVelocity * airTime
    val distanceToGoal = futureShooterToGoal.norm
    val ballSpeed = DISTANCE_TO_BALL_SPEED.get(distanceToGoal)
    val targetPitch = DISTANCE_TO_HOOD_ANGLE.get(distanceToGoal)
    val targetYaw = if (AllianceColor.isRed()) {
        flip(futureShooterToGoal.angle)
    } else {
        futureShooterToGoal.angle
    }
    Logger.recordOutput("ShotCalcs/DistanceToGoal(m)", distanceToGoal)
    Logger.recordOutput("ShotCalcs/Iterations", *iterations)
    Logger.recordOutput("ShotCalcs/FutureShooterToGoal", Translation2d.struct, futureShooterToGoal)

    if (RobotMode.isSim() || DEBUG_MODE) {
        val launchAngle = Rotation3d(0.0, -targetPitch, futureShooterToGoal.angle.radians)
        simulateShot(
            pose = Translation3d(
                fieldOriginToShooter.x,
                fieldOriginToShooter.y,
                Shooter.FUEL_LAUNCH_HEIGHT.`in`(Meters)
            ),
            velocity = Translation3d(ballSpeed, launchAngle) +
                Translation3d(shooterVelocity.x, shooterVelocity.y, 0.0),
            dt = 0.01
        )
    }

    return Shooter.Setpoint(
        targetYaw,
        Rotation2d.fromRadians(targetPitch),
        BALL_TO_FLYWHEEL_SPEED.get(ballSpeed)
    )
}

private const val NEWTONS_METHOD_ITERATIONS = 7
private const val DEBUG_MODE = false
private val HUB_LOCATION = FieldConstants.Hub.topCenterPoint.toTranslation2d()
private val LOOKAHEAD_SECS = Tunable.of("ShotCalcs/Lookahead time(secs)", 0.1)
// Because of energy loss, the linear velocity of the ball isn't exactly
// omega * r; so, we compensate for that with a lerp table.
private val BALL_TO_FLYWHEEL_SPEED =
    TunableLerpTable("ShotCalcs/Ball Speed(mps) -> Flywheel Speed(rad per s)")
        .put(0.0, 0.0)
        .put(14.5, 285.433)
private val DISTANCE_TO_BALL_SPEED =
    TunableLerpTable("ShotCalcs/Distance(m) -> Ball Velocity(mps)")
        .put(1.3, 6.5)
        .put(1.5, 6.4)
        .put(1.7, 6.28)
        .put(2.1, 6.45)
        .put(2.65, 6.8)
        .put(2.95, 7.05)
        .put(3.05, 7.2)
        .put(3.75, 7.7)
        .put(4.35, 8.05)
        .put(4.65, 8.5)
        .put(5.0, 8.5)
        .put(5.226, 8.7)
        .put(5.71, 8.95)
        .put(6.371, 9.5)
        .put(6.7, 9.75)
        .put(7.04, 10.0)
        .put(7.5, 10.5)
        .put(8.0, 10.7)
private val DISTANCE_TO_HOOD_ANGLE =
    TunableLerpTable("ShotCalcs/Distance(m) -> Hood Angle(rad)")
        .put(1.6, 1.22)
        .put(2.1, 1.14)
        .put(2.65, 1.06)
        .put(2.95, 1.0)
        .put(3.35, 0.96)
        .put(4.35, 0.88)
        .put(4.65, 0.78)
        .put(5.0, 1.0)
        .put(8.0, 1.0)
private val DISTANCE_TO_AIR_TIME =
    TunableLerpTable("ShotCalcs/Distance(m) -> Air Time(s)")
        .put(0.0, 0.0)
        .put(1.5, 0.872)
        .put(1.6, 0.858)
        .put(1.7, 0.842)
        .put(1.8, 0.818)
        .put(1.9, 0.8)
        .put(2.2, 0.798)
        .put(2.3, 0.8)
        .put(2.4, 0.804)
        .put(2.5, 0.81)
        .put(2.6, 0.812)
        .put(2.7, 0.816)
        .put(2.8, 0.818)
        .put(3.1, 0.814)
        .put(3.2, 0.842)
        .put(3.3, 0.848)
        .put(3.4, 0.852)
        .put(3.5, 0.858)
        .put(3.6, 0.864)
        .put(3.7, 0.87)
        .put(3.8, 0.876)
        .put(3.9, 0.882)
        .put(4.0, 0.884)
        .put(4.1, 0.886)
        .put(4.2, 0.888)
        .put(4.4, 0.89)
        .put(4.6, 0.87)
        .put(4.7, 0.844)
        .put(4.8, 0.814)
        .put(4.9, 0.922)
        .put(5.0, 1.018)
        .put(5.1, 1.102)
        .put(5.2, 1.15)
        .put(5.3, 1.168)
        .put(5.4, 1.182)
        .put(5.5, 1.192)
        .put(5.6, 1.202)
        .put(5.7, 1.212)
        .put(5.8, 1.222)
        .put(5.9, 1.234)
        .put(6.0, 1.25)
        .put(6.1, 1.264)
        .put(6.2, 1.28)
        .put(6.3, 1.294)
        .put(6.4, 1.31)
        .put(6.5, 1.324)
        .put(6.6, 1.338)
        .put(6.7, 1.352)
        .put(6.8, 1.366)
        .put(6.9, 1.378)
        .put(7.0, 1.39)
        .put(7.1, 1.404)
        .put(7.2, 1.416)
        .put(7.3, 1.436)
        .put(7.4, 1.454)
        .put(7.5, 1.472)
        .put(7.6, 1.49)
        .put(7.7, 1.502)
        .put(7.8, 1.51)
        .put(7.9, 1.516)

/**
 * A (sort of jank) way to generate entries to the DISTANCE_TO_AIR_TIME table
 * using the fuel shot simulator. Should only be used when simulating the code.
 */
fun printDistanceToAirTimeTable() {
    if (!RobotMode.isSim()) return
    var prevRoundedAirTime = -1.0
    repeat(80) { i ->
        val dist = i * 0.1
        var fieldOriginToShooter = Translation2d(HUB_LOCATION.x - dist, HUB_LOCATION.y)
        fieldOriginToShooter += Shooter.ROBOT_TO_LAUNCH_POINT.translation
        val shooterToGoal = HUB_LOCATION - fieldOriginToShooter
        val ballSpeed = DISTANCE_TO_BALL_SPEED.get(shooterToGoal.norm)
        val targetPitch = DISTANCE_TO_HOOD_ANGLE.get(shooterToGoal.norm)
        val launchAngle = Rotation3d(0.0, -targetPitch, shooterToGoal.angle.radians)
        val airTime = simulateShot(
            pose = Translation3d(
                fieldOriginToShooter.x,
                fieldOriginToShooter.y,
                Shooter.FUEL_LAUNCH_HEIGHT.`in`(Meters)
            ),
            velocity = Translation3d(ballSpeed, launchAngle),
            dt = 0.002
        )
        val roundedDist = (dist * 100000.0).roundToInt() / 100000.0
        val roundedAirTime = (airTime * 100000.0).roundToInt() / 100000.0
        if (roundedAirTime == prevRoundedAirTime) return@repeat
        prevRoundedAirTime = roundedAirTime
        println(".put($roundedDist, $roundedAirTime)")
    }
}