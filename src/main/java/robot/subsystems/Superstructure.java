package robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.AllianceColor;
import lib.Tunable;
import lib.commands.CmdSequence;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.intake.GroundIntake;
import robot.subsystems.serializer.Serializer;
import robot.subsystems.shooter.Shooter.Target;
import robot.subsystems.shooter.ShotCalcsKt;
import robot.subsystems.shooter.Shooter;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import static choreo.util.ChoreoAllianceFlipUtil.flip;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static lib.commands.CmdLogger.logged;

@SuppressWarnings("unused")
@RequiredArgsConstructor
public class Superstructure {
    // Constants
    private static final Tunable<Double>
        YAW_TOLERANCE = Tunable.of("ShotCalcs/Aiming/Tolerance (rad)", 0.15),
        SOTM_TOLERANCE_MULTIPLIER = Tunable.of("ShotCalcs/SOTM tolerance multiplier (scalar)", 0.2);

    // Constructor Parameters
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

    @AutoLogOutput(key = "ShotCalcs/YawErrorRadians")
    private double yawError = 0.0;

    /** Whether the shot trajectory is good enough to run the serializer. */
    @AutoLogOutput
    public boolean canSerialize() {
        double robotSpeed = Math.hypot(
            drive.getRobotSpeeds().vxMetersPerSecond,
            drive.getRobotSpeeds().vyMetersPerSecond
        );
        double toleranceMultiplier = 1.0 + robotSpeed * SOTM_TOLERANCE_MULTIPLIER.get();
        return Math.abs(yawError) < YAW_TOLERANCE.get()
            && shotSetpoint.isPossible()
            // increase shooter tolerance when running shoot-on-the-move
            && shooter.atGoal(toleranceMultiplier);
    }

    private void resetState() {
        shotSetpoint = Shooter.Setpoint.NULL;
        shotTarget = Optional.empty();
        rotationOverride = Optional.empty();
        yawError = 0.0;
    }

    // Commands
    public Command spinupAndAimCmd(Target target) {
        var cmd = shooter.setVelocityCmd(() -> {
            shotSetpoint = ShotCalcsKt.getHubShotSetpoint(
                drive.getPose(), drive.getFieldSpeeds(), shooter, true, target
            );
            shotTarget = Optional.of(target);
            rotationOverride = Optional.of(shotSetpoint.yaw());
            yawError = MathUtil.angleModulus(shotSetpoint.yaw().minus(drive.getPose().getRotation()).getRadians());
            return RadiansPerSecond.of(shotSetpoint.radPerSec() * speedAdjustment.getAsDouble());
        })
            .finallyDo(this::resetState);
        return logged(cmd, "SpinupAndAim(" + target + ")");
    }

    public Command shootInAutoCmd(Target target) {
        return Commands.parallel(
            spinupAndAimCmd(target),
            CmdSequence.of(
                Commands.waitUntil(this::canSerialize),
                Commands.waitSeconds(0.3),
                serializer.runCmd()
            )
        );
    }

    public Command spinupCmd(Target target, Pose2d blueTargetPose) {
        var speeds = new ChassisSpeeds();
        var cmd = shooter.setVelocityCmd(() -> {
            var pose = AllianceColor.isRed() ? flip(blueTargetPose) : blueTargetPose;
            shotSetpoint = ShotCalcsKt.getHubShotSetpoint(
                drive.getPose(), drive.getFieldSpeeds(), shooter, true, target
            );
            shotTarget = Optional.of(target);
            return RadiansPerSecond.of(shotSetpoint.radPerSec());
        })
            .finallyDo(this::resetState);
        return logged(cmd, "SpinupAndAim(" + target + ")");
    }

    public Command manualHubShotCmd() {
        return CmdSequence.of(
            Commands.runOnce(() -> {
                var pose = new Pose2d(1.45, 4.05, Rotation2d.kZero);
                drive.resetPose(AllianceColor.isRed() ? flip(pose) : pose);
            }),
            spinupAndAimCmd(Target.HUB)
        );
    }
}
