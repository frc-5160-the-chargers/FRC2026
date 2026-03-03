package robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.AllianceColor;
import lib.Convert;
import lib.Tunable;
import lib.commands.NonBlockingCmds;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import robot.controllers.DriverController;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.intake.GroundIntake;
import robot.subsystems.serializer.Serializer;
import robot.subsystems.shooter.ShotCalcsKt;
import robot.subsystems.shooter.Shooter;

import java.util.List;
import java.util.Optional;

import static choreo.util.ChoreoAllianceFlipUtil.flip;
import static lib.commands.CmdLogger.logged;

@SuppressWarnings("unused")
@RequiredArgsConstructor
public class Superstructure {
    // Constants
    private static final List<Rectangle2d> HUB_NO_SHOOT_ZONES = List.of(
        new Rectangle2d(new Translation2d(4.3, 0), new Translation2d(12.3, 8.1))
    );
    private static final Tunable<Double>
        YAW_TOLERANCE = Tunable.of("HubAiming/Tolerance (rad)", 0.15),
        FERRY_HOOD_ANGLE = Tunable.of("Ferrying/HoodAngle (rad)", 65 * Convert.DEGREES_TO_RADIANS);

    // Constructor Parameters
    private final DriverController controller;
    private final SwerveSubsystem drive;
    private final GroundIntake groundIntake;
    private final Shooter shooter;
    private final Serializer serializer;

    // State
    /** A rotation override used for shooting. */
    @Getter
    private Optional<Rotation2d> rotationOverride = Optional.empty();

    @AutoLogOutput(key = "ShotCalcs/Setpoint")
    private Shooter.Setpoint shotSetpoint = Shooter.Setpoint.NULL;

    @AutoLogOutput(key = "Swerve/YawErr")
    private Rotation2d yawErr = Rotation2d.kZero;

    private double swerveNetSpeed = 0.0;

    private void updateHubShotSetpoint(Pose2d pose, ChassisSpeeds speeds) {
        shotSetpoint = ShotCalcsKt.getHubShotSetpoint(pose, speeds, shooter, true);
        yawErr = drive.getPose().getRotation().minus(shotSetpoint.yaw());
        swerveNetSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    private void updateHubShotSetpoint() {
        updateHubShotSetpoint(
            drive.getSim().getSimulatedDriveTrainPose(), drive.getFieldSpeeds()
        );
    }

    private void resetShotSetpoint() {
        shotSetpoint = Shooter.Setpoint.NULL;
        rotationOverride = Optional.empty();
    }

    private Command runSerializerCmd(List<Rectangle2d> noShootZones) {
        return NonBlockingCmds.sequence(
            Commands.waitUntil(() -> shooter.atGoal(1.0)),
            serializer.runWhileTrueCmd(() -> {
                if (!shotSetpoint.isPossible()) return false;
                double yawTolerance = YAW_TOLERANCE.get() * (1 + swerveNetSpeed);
                if (Math.abs(yawErr.getRadians()) > yawTolerance) return false;
                for (var zone: noShootZones) {
                    if (zone.contains(drive.getPose().getTranslation())) return false;
                }
                return true;
            })
        );
    }

    // Commands
    public Command shootInHubCmd() {
        var cmd = NonBlockingCmds.parallel(
            shooter.targetingCmd(() -> {
                updateHubShotSetpoint();
                rotationOverride = Optional.of(shotSetpoint.yaw());
                return shotSetpoint;
            }),
            runSerializerCmd(HUB_NO_SHOOT_ZONES)
        )
            .finallyDo(this::resetShotSetpoint);
        return logged(cmd, "ShootAtHub");
    }

    public Command spinupForHubShotCmd(Pose2d blueTargetPose) {
        var cmd = shooter.targetingCmd(() -> {
            var pose = AllianceColor.isRed() ? flip(blueTargetPose) : blueTargetPose;
            updateHubShotSetpoint(blueTargetPose, new ChassisSpeeds());
            return shotSetpoint;
        })
            .finallyDo(this::resetShotSetpoint);
        return logged(cmd, "SpinupForHubShot");
    }

    public Command ferryCmd() {
        var cmd = shooter.targetingCmd(() -> {
            shotSetpoint = new Shooter.Setpoint(
                Rotation2d.kZero,
                Rotation2d.fromRadians(FERRY_HOOD_ANGLE.get()),
                0.0,
                true
            );
            return shotSetpoint;
        });
        return logged(cmd, "Ferry");
    }

    public Command shootInHubFromWallCmd() {
        return Commands.none();
    }

    public Command simpleFerryCmd() {
        return Commands.none();
    }
}
