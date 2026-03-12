package robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.AllianceColor;
import lib.Tunable;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.intake.GroundIntake;
import robot.subsystems.serializer.Serializer;
import robot.subsystems.shooter.Shooter.Target;
import robot.subsystems.shooter.ShotCalcsKt;
import robot.subsystems.shooter.Shooter;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static choreo.util.ChoreoAllianceFlipUtil.flip;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static lib.commands.CmdLogger.logged;

@SuppressWarnings("unused")
@RequiredArgsConstructor
public class Superstructure {
    // Constants
    private static final List<Rectangle2d>
        HUB_NO_SHOOT_ZONES = List.of(
            new Rectangle2d(new Translation2d(4.3, 0), new Translation2d(12.3, 8.1))
        ),
        FERRY_NO_SHOOT_ZONES = List.of(
            new Rectangle2d(new Translation2d(4.6, 3.05), new Translation2d(11.9, 5.0))
        );
    private static final Tunable<Double> YAW_TOLERANCE = Tunable.of("HubAiming/Tolerance (rad)", 0.15);
    private static final Tunable<Boolean> rotationEnabled = Tunable.of("HubAiming/Rotation Enabled", true);

    // Constructor Parameters
    @AutoLogOutput private final BooleanSupplier serializeOverride;
    private final DoubleSupplier speedAdjustment;
    private final SwerveSubsystem drive;
    private final GroundIntake groundIntake;
    private final Shooter shooter;
    private final Serializer serializer;

    // State
    /** A rotation override used for shooting. */
    @Getter
    private Optional<Rotation2d> rotationOverride = Optional.empty();

    /** The current shooting target of the robot. (Hub or Ground) */
    @Getter
    private Optional<Target> shotTarget = Optional.empty();

    @AutoLogOutput(key = "ShotCalcs/Setpoint")
    private Shooter.Setpoint shotSetpoint = Shooter.Setpoint.NULL;

    private void updateState(Target target, Pose2d pose, ChassisSpeeds speeds) {
        shotTarget = Optional.of(target);
        shotSetpoint = ShotCalcsKt.getHubShotSetpoint(pose, speeds, shooter, true, target);
    }

    private void resetState() {
        shotSetpoint = Shooter.Setpoint.NULL;
        shotTarget = Optional.empty();
        rotationOverride = Optional.empty();
    }

    private boolean shouldSerialize(List<Rectangle2d> noShootZones) {
        if (DriverStation.isTeleopEnabled()) return serializeOverride.getAsBoolean();
        if (!shotSetpoint.isPossible()) return false;
        for (var zone: noShootZones) {
            if (zone.contains(drive.getPose().getTranslation())) return false;
        }
        return true;
    }

    // Commands
    public Command shootCmd(Target target, boolean shouldAim, boolean shouldPulse) {
        var noShootZones = target == Target.HUB ? HUB_NO_SHOOT_ZONES : FERRY_NO_SHOOT_ZONES;
        var serializeCmd = shouldPulse
            ? serializer.pulseCmd(() -> shouldSerialize(List.of()))
            : serializer.runCmd(() -> shouldSerialize(List.of()));
        var cmd = Commands.parallel(
            shooter.setVelocityCmd(() -> {
                updateState(target, drive.getPose(), drive.getFieldSpeeds());
                if (rotationEnabled.get()) rotationOverride = Optional.of(shotSetpoint.yaw());
                return RadiansPerSecond.of(shotSetpoint.radPerSec() * speedAdjustment.getAsDouble());
            }),
            Commands.sequence(
                Commands.waitUntil(() -> shooter.atGoal(1.0)),
                Commands.waitSeconds(1.0),
                serializeCmd
            )
        )
            .finallyDo(this::resetState);
        return logged(cmd, "ShootAtHub");
    }

    public Command shootCmd(Target target) {
        return shootCmd(target, true, false);
    }

    public Command hubAimCmd() {
        var cmd = Commands.run(() -> {
            updateState(Target.HUB, drive.getPose(), drive.getFieldSpeeds());
            rotationOverride = Optional.of(shotSetpoint.yaw());
        });
        return logged(cmd, "AimAtHub");
    }

    public Command spinupForHubShotCmd(Pose2d blueTargetPose) {
        var speeds = new ChassisSpeeds();
        var cmd = shooter.setVelocityCmd(() -> {
            var pose = AllianceColor.isRed() ? flip(blueTargetPose) : blueTargetPose;
            updateState(Target.HUB, blueTargetPose, speeds);
            return RadiansPerSecond.of(shotSetpoint.radPerSec() * speedAdjustment.getAsDouble());
        })
            .finallyDo(this::resetState);
        return logged(cmd, "SpinupForHubShot");
    }

    public Command visionlessHubShotCmd() {
        var resetPoseCmd = Commands.runOnce(() -> {
            var pose = new Pose2d(1.45, 4.05, Rotation2d.kZero);
            drive.resetPose(AllianceColor.isRed() ? flip(pose) : pose);
        });
        return logged(resetPoseCmd, "ResetPoseForVisionlessHubShot")
            .andThen(shootCmd(Target.HUB));
    }
}
