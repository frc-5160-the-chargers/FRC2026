@file:Suppress("LocalVariableName")
package robot.subsystems.shooter.math

import choreo.util.ChoreoAllianceFlipUtil.flip
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.*
import lib.AllianceColor
import lib.RobotMode
import lib.Tunable
import lib.TunableLerpTable
import org.littletonrobotics.junction.Logger
import robot.constants.FieldConstants
import robot.subsystems.shooter.DataTypes.ShooterSetpoint
import robot.subsystems.shooter.KrakenFlywheels.FlywheelData
import robot.subsystems.shooter.Shooter.*

private val HUB_SHOT_MAP = HubShotMap()
private val HUB_LOCATION = FieldConstants.Hub.topCenterPoint.toTranslation2d()
private const val NEWTONS_METHOD_ITERATIONS = 20
private val FLYWHEEL_SPEED_TOLERANCE_MPS = Tunable.of("Shooter/Speed Tolerance (MPS)", 0.05)
private val LOOKAHEAD_SECS = Tunable.of("Shooter/Lookahead time(secs)", 0.0)
private val ALWAYS_MAXIMIZE_VEL_DIST = Tunable.of("Shooter/Always maximize velocity dist (M)", 10.0)
// Because of energy loss, the linear velocity of the ball isn't exactly
// omega * r; so, we compensate for that with a lerp table.
private val ANGULAR_TO_LINEAR_VEL = TunableLerpTable("Shooter/AngularVel(rad per sec) to LinearVel(mps)")
    .put(0.0, 0.0)
    .put(285.433, 14.5)

/** Calculates the desired shot parameters to launch fuel into the hub. */
fun calcHubShotSetpoint(
    robotPose: Pose2d,
    fieldCentricVel: ChassisSpeeds,
    flywheelInputs: FlywheelData,
    speedDebouncer: Debouncer,
    prevSetpoint: ShooterSetpoint
): ShooterSetpoint {
    val goalPosition = if (AllianceColor.isRed()) flip(HUB_LOCATION) else HUB_LOCATION
    val iterations = Array(NEWTONS_METHOD_ITERATIONS + 1) { Pose2d.kZero }
    // In order to compensate for processing delay, we don't use the robot's pose directly;
    // rather, we use the robot pose a small amount of time(0.05 secs-ish) into the future.
    // We multiply this time by the robot's velocity in the x, y, and omega directions,
    // and then add it to each component of the robot's pose to get the new "future" pose.
    val lookaheadSecs = LOOKAHEAD_SECS.get()
    val vx = fieldCentricVel.vxMetersPerSecond
    val vy = fieldCentricVel.vyMetersPerSecond
    val omega = fieldCentricVel.omegaRadiansPerSecond

    val lookaheadComp = Transform2d(vx, vy, Rotation2d.fromRadians(omega)) * lookaheadSecs
    val fieldOriginToShooter = (robotPose + lookaheadComp + ROBOT_TO_FUEL_LAUNCH_POINT).translation
    val shooterToGoal = goalPosition - fieldOriginToShooter
    iterations[0] = Pose2d(fieldOriginToShooter, shooterToGoal.angle)

    val shooterVelocity = Translation2d(
        vx + omega * ROBOT_TO_FUEL_LAUNCH_POINT.x,
        vy - omega * ROBOT_TO_FUEL_LAUNCH_POINT.y
    )
    val ballSpeed = ANGULAR_TO_LINEAR_VEL.get(flywheelInputs.velocity.`in`(RadiansPerSecond))
    // The time it takes for a ball to reach the goal.
    val airTime = 0.0
//    var airTime = HUB_SHOT_MAP.get(shooterToGoal.norm, ballSpeed).airTimeSecs
//    repeat(NEWTONS_METHOD_ITERATIONS) { i ->
//        // In this case, futureShooterToGoal is the x and y distance between the shooter and the goal
//        // by the time the ball has already landed within the goal.
//        val futureFieldOriginToShooter = fieldOriginToShooter + shooterVelocity * airTime
//        val futureShooterToGoal = goalPosition - futureFieldOriginToShooter
//        val distanceToGoal = futureShooterToGoal.norm
//        iterations[i + 1] = Pose2d(futureFieldOriginToShooter, futureShooterToGoal.angle)
//
//        // Newton's method
//        val error = airTime - HUB_SHOT_MAP.get(distanceToGoal, ballSpeed).airTimeSecs
//        val dt_dD = HUB_SHOT_MAP.getAirTimeDerivative(futureShooterToGoal.norm, ballSpeed)
//        val dError_dt = 1 + dt_dD * futureShooterToGoal.dot(shooterVelocity) / distanceToGoal
//        airTime -= error / dError_dt
//    }
    val futureShooterToGoal = shooterToGoal - shooterVelocity * airTime
    val targetPitch = Rotation2d.fromRadians(HUB_SHOT_MAP.get(futureShooterToGoal.norm, ballSpeed).pitchRad)
    Logger.recordOutput("Shooter/DistanceToTarget", futureShooterToGoal.norm)
    Logger.recordOutput("ShooterCalcs/Iterations", *iterations)
    Logger.recordOutput("ShooterCalcs/FutureShooterToGoal", futureShooterToGoal)
    // TODO resolve
    Logger.recordOutput("BaseRobotToShooter", ROBOT_TO_FUEL_LAUNCH_POINT)
    Logger.recordOutput("SupposedExtraToCompute", fieldOriginToShooter - robotPose.translation)

    val minVelocity = HUB_SHOT_MAP.minVelocityAt(futureShooterToGoal.norm)
    val maxVelocity = HUB_SHOT_MAP.maxVelocityAt(futureShooterToGoal.norm)
    val tolerance = FLYWHEEL_SPEED_TOLERANCE_MPS.get()
    val speedTooLow = speedDebouncer.calculate(ballSpeed < minVelocity - tolerance)
    val speedTooHigh = speedDebouncer.calculate(ballSpeed > maxVelocity + tolerance)
    val flywheelRadPerSec = if (speedTooHigh || futureShooterToGoal.norm < ALWAYS_MAXIMIZE_VEL_DIST.get()) {
        maxVelocity
    } else if (speedTooLow) {
        minVelocity
    } else {
        prevSetpoint.speedRadPerSec
    }

    if (RobotMode.isSim()) {
        simulateShot(
            pose = Translation3d(
                fieldOriginToShooter.x,
                fieldOriginToShooter.y,
                FUEL_LAUNCH_HEIGHT.`in`(Meters)
            ),
            velocity = Translation3d(
                clamp(ballSpeed, minVelocity, maxVelocity),
                Rotation3d(0.0, -targetPitch.radians, futureShooterToGoal.angle.radians)
            )
        )
    }

    return ShooterSetpoint(
        futureShooterToGoal.angle,
        targetPitch,
        flywheelRadPerSec,
        !speedTooLow && !speedTooHigh
    )
}