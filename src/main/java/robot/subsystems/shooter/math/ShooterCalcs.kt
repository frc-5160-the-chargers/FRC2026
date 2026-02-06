// We use kotlin to write our shooter calculations code because it has operator overloading (w)
package robot.subsystems.shooter.math

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import robot.subsystems.shooter.ShooterConsts.*
import kotlin.math.exp

fun computeSetpoint(
    target: Translation2d,
    shotMap: ShotMap,
    robotPose: Pose2d,
    fieldCentricVel: ChassisSpeeds,
    currentShooterVel: AngularVelocity
): ShooterSetpoint {
    val fieldOriginToShooter = (robotPose + ROBOT_TO_SHOOTER).translation
    val robotToShooter = fieldOriginToShooter - robotPose.translation
    var shooterToTarget = target - fieldOriginToShooter

    val omega = fieldCentricVel.omegaRadiansPerSecond
    val shooterVx = fieldCentricVel.vxMetersPerSecond + omega * robotToShooter.x
    val shooterVy = fieldCentricVel.vyMetersPerSecond - omega * robotToShooter.y
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