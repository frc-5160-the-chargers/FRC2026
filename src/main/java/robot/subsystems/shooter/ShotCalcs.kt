// Why do we use kotlin here? Well, kotlin has this feature called operator overloading,
// where you can call operators like +, *, >, >=, etc on non-numbers. This makes
// doing math stuff about 10x clearer when it's done in kotlin vs. java.
@file:Suppress("LocalVariableName", "Unused")
package robot.subsystems.shooter

import choreo.util.ChoreoAllianceFlipUtil.flip
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.RadiansPerSecond
import lib.AllianceColor
import lib.RobotMode
import lib.Tunable
import lib.TunableLerpTable
import org.littletonrobotics.junction.Logger
import robot.constants.FieldConstants
import robot.subsystems.shooter.Shooter.Target
import java.io.BufferedWriter
import java.io.FileWriter
import kotlin.math.roundToInt

fun initialize() {}

/** Calculates the desired shot parameters to launch fuel into the specified target. */
fun getShotSetpoint(
    robotPose: Pose2d,
    fieldCentricVel: ChassisSpeeds,
    shooter: Shooter,
    simulateShot: Boolean,
    target: Target
): Shooter.Setpoint {
    // a when statement is equivalent to a java switch statement.
    var goalPosition = when (target) {
        Target.GROUND -> if (robotPose.y > 4) PASS_LOC_TOP else PASS_LOC_BOTTOM
        Target.HUB -> HUB_LOCATION
    }
    val distanceToSpeed = when (target) {
        Target.GROUND -> DISTANCE_TO_BALL_SPEED_GROUND
        Target.HUB -> DISTANCE_TO_BALL_SPEED_HUB
    }
    val distanceToAirTime = when (target) {
        Target.GROUND -> DISTANCE_TO_AIR_TIME_GROUND
        Target.HUB -> DISTANCE_TO_AIR_TIME_HUB
    }
    if (AllianceColor.isRed()) goalPosition = flip(goalPosition)
    // equivalent to Pose2d[] iterations = new Pose2d[NEWTONS_METHOD_ITERATIONS + 1];
    val iterations = Array(NEWTONS_METHOD_ITERATIONS + 1) { Pose2d.kZero }
    val vx = fieldCentricVel.vxMetersPerSecond
    val vy = fieldCentricVel.vyMetersPerSecond
    val omega = fieldCentricVel.omegaRadiansPerSecond

    // In order to compensate for processing delay, we don't use the robot's pose directly;
    // rather, we use the robot pose a small amount of time(0.05 secs-ish) into the future.
    // We multiply this time by the robot's velocity in the x, y, and omega directions,
    // and then add it to each component of the robot's pose to get the new "future" pose.
    val lookaheadComp = Transform2d(vx, vy, Rotation2d.fromRadians(omega)) * LOOKAHEAD_SECS.get()
    val lookaheadPose = robotPose + lookaheadComp
    // The x and y distance from the field origin to the launch point.
    val fieldOriginToShooter = (lookaheadPose + Shooter.ROBOT_TO_LAUNCH_POINT).translation
    // The x and y distance from the launch point to the hub.
    val shooterToGoal = goalPosition - fieldOriginToShooter
    iterations[0] = Pose2d(fieldOriginToShooter, shooterToGoal.angle)
    // The "norm" is basically sqrt(x^2 + y^2) (or the pythagorean theorem)
    Logger.recordOutput("ShotCalcs/InitialTargetDist(m)", shooterToGoal.norm)

    val botToLaunchPointRotated = fieldOriginToShooter - robotPose.translation
    val shooterVelocity = Translation2d(
        vx - omega * botToLaunchPointRotated.y,
        vy + omega * botToLaunchPointRotated.x
    )
    // The time it takes for a ball to reach the goal.
    var airTime = 0.0
    repeat(NEWTONS_METHOD_ITERATIONS) { i ->
        // In this case, futureShooterToGoal is the x and y distance between the shooter and the goal
        // by the time the ball has already landed within the goal.
        val futureFieldOriginToShooter = fieldOriginToShooter + shooterVelocity * airTime
        val futureShooterToGoal = goalPosition - futureFieldOriginToShooter
        val distanceToGoal = futureShooterToGoal.norm
        iterations[i + 1] = Pose2d(futureFieldOriginToShooter, futureShooterToGoal.angle)

        // Newton's method: see https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/newton-shooting.html
        val error = airTime - distanceToAirTime.get(distanceToGoal)
        val dt_dD = 1 / distanceToSpeed.get(distanceToGoal) // just an estimate
        val dD_dt = -futureShooterToGoal.dot(shooterVelocity) / distanceToGoal
        val dError_dt = 1 - dt_dD * dD_dt
        airTime -= error / dError_dt
    }
    val futureShooterToGoal = shooterToGoal - shooterVelocity * airTime
    val targetDist = futureShooterToGoal.norm
    val ballSpeed = distanceToSpeed.get(targetDist)
    val targetYaw = futureShooterToGoal.angle
    Logger.recordOutput("ShotCalcs/BallSpeed", ballSpeed)
    Logger.recordOutput("ShotCalcs/FutureShooterToGoal", Translation2d.struct, futureShooterToGoal)
    Logger.recordOutput("ShotCalcs/TargetDistance(m)", targetDist)
    Logger.recordOutput("ShotCalcs/Iterations", *iterations)

    if (simulateShot && RobotMode.isSim()) {
        val launchAngle = Rotation3d(0.0, -Shooter.LAUNCH_ANGLE.radians, robotPose.rotation.radians)
        val angularVel = BALL_TO_FLYWHEEL_SPEED.getKey(shooter.velocity().`in`(RadiansPerSecond))
        simulateShot(
            pose = Translation3d(
                fieldOriginToShooter.x,
                fieldOriginToShooter.y,
                Shooter.FUEL_LAUNCH_HEIGHT.`in`(Meters)
            ),
            velocity = Translation3d(angularVel, launchAngle) +
                Translation3d(shooterVelocity.x, shooterVelocity.y, 0.0),
            dt = 0.05
        )
    }

    return Shooter.Setpoint(
        targetYaw,
        BALL_TO_FLYWHEEL_SPEED.get(ballSpeed),
        targetDist >= MIN_DISTANCE_TO_SHOOT.get()
    )
}

private const val NEWTONS_METHOD_ITERATIONS = 7
private val HUB_LOCATION = FieldConstants.Hub.topCenterPoint.toTranslation2d()
private val PASS_LOC_TOP = Translation2d(1.3, 6.0)
private val PASS_LOC_BOTTOM = Translation2d(1.3, 2.0)
private val MIN_DISTANCE_TO_SHOOT = Tunable.of("ShotCalcs/Minimum Distance to Shoot(m)", 1.5)
private val LOOKAHEAD_SECS = Tunable.of("ShotCalcs/Lookahead time(secs)", 0.0)
//private val DRAG_COMP_INVERSE_SECS = Tunable.of("ShotCalcs/Drag Compensation(secs ^ -1)", 0.02)
// Because of energy loss, the linear velocity of the ball isn't exactly
// omega * r; so, we compensate for that with a lerp table.
private val BALL_TO_FLYWHEEL_SPEED =
    TunableLerpTable("ShotCalcs/Ball Speed(mps) -> Flywheel Speed(rad per s)")
        .put(6.0, 245.0)
        .put(7.8, 315.0)
        .put(8.5, 328.0)
        .put(12.0, 460.0)
private val DISTANCE_TO_BALL_SPEED_HUB =
    TunableLerpTable("ShotCalcs/(Hub) Distance(m) -> Ball Velocity(mps)")
        .put(1.7, 6.2)
        .put(2.1, 6.6)
        .put(2.65, 7.1)
        .put(2.95, 7.4)
        .put(3.05, 7.5)
        .put(3.5, 8.1)
        .put(3.75, 8.5)
        .put(4.35, 9.0)
        .put(4.65, 9.3)
        .put(5.0, 9.7)
        .put(5.2, 10.1)
        .put(5.7, 10.5)
        .put(6.3, 11.1)
        .put(6.7, 11.4)
        .put(7.0, 11.7)
        .put(7.5, 12.1)
        .put(8.0, 12.7)

private val DISTANCE_TO_BALL_SPEED_GROUND =
    TunableLerpTable("ShotCalcs/(Ground) Distance(m) -> Ball Velocity(mps)")
        .put(0.0, 0.0)
        .put(4.5, 8.0)
        .put(6.0, 9.5)
        .put(7.0, 10.5)
        .put(8.0, 12.0)


/**
 * Generates the DistanceToAirTime.kt file (and the DISTANCE_TO_AIR_TIME table)
 * by using the fuel shot simulator to theoretically calculate
 * the air time of the ball at various distances from the hub.
 */
private fun writeDistanceToAirTimeTable(target: Target): String {
    val goalPosition = when (target) {
        Target.GROUND -> PASS_LOC_TOP
        Target.HUB -> HUB_LOCATION
    }
    val distanceToSpeed = when (target) {
        Target.GROUND -> DISTANCE_TO_BALL_SPEED_GROUND
        Target.HUB -> DISTANCE_TO_BALL_SPEED_HUB
    }
    val text = StringBuilder()
    text.append("val DISTANCE_TO_AIR_TIME_$target =\n")
    text.append("    TunableLerpTable(\"ShotCalcs/Distance(m) -> Air Time(s)\")\n")
    repeat(80) { i ->
        val dist = i * 0.1
        var fieldOriginToShooter = Translation2d(goalPosition.x - dist, goalPosition.y)
        fieldOriginToShooter += Shooter.ROBOT_TO_LAUNCH_POINT.translation
        val shooterToGoal = HUB_LOCATION - fieldOriginToShooter
        val ballSpeed = distanceToSpeed.get(shooterToGoal.norm)
        val launchAngle = Rotation3d(0.0, -Shooter.LAUNCH_ANGLE.radians, shooterToGoal.angle.radians)
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
        // If the air time is < 0.5, the most likely case is that the shot is impossible;
        // thus, we shouldn't record data for it.
        if (roundedAirTime < 0.5) return@repeat
        text.append("        .put($roundedDist, $roundedAirTime)\n")
    }
    return text.toString()
}

fun main() {
    val path = "src/main/java/robot/subsystems/shooter/DistanceToAirTime.kt"
    val fileWriter = BufferedWriter(FileWriter(path))
    fileWriter.write("// This file was autogenerated by ShotCalcs.kt.\n")
    fileWriter.write("package robot.subsystems.shooter\n\n")
    fileWriter.write("import lib.TunableLerpTable\n\n")
    fileWriter.write(writeDistanceToAirTimeTable(Target.HUB))
    fileWriter.write(writeDistanceToAirTimeTable(Target.GROUND))
    fileWriter.close()
}