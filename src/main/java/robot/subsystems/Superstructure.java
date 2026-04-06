package robot.subsystems;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.AllianceColor;
import lib.Tunable;
import lib.commands.CmdSequence;
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
import java.util.function.Supplier;

import static choreo.util.ChoreoAllianceFlipUtil.flip;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static lib.commands.CmdLogger.logged;

@SuppressWarnings("unused")
@RequiredArgsConstructor
public class Superstructure {
    // Constants
    private static final Tunable<Double>
        YAW_TOLERANCE = Tunable.of("ShotCalcs/Aiming/Tolerance (rad)", 0.15),
        SOTM_TOLERANCE_MULTIPLIER = Tunable.of("ShotCalcs/SOTM tolerance multiplier (scalar)", 0.2),
        SOTM_DESIRED_SPEEDS_WEIGHT = Tunable.of("ShotCalcs/SOTM Desired Speeds Weight", 0.85);

    // Constructor Parameters
    private final DoubleSupplier speedAdjustment;
    private final Supplier<ChassisSpeeds> desiredSpeedsSupplier;
    private final SwerveSubsystem drive;
    private final GroundIntake intake;
    private final Shooter shooter;
    private final Serializer serializer;

    // State
    /** A rotation override used for shooting. */
    public Optional<Rotation2d> rotationOverride = Optional.empty();

    /** Whether the robot wants to shoot at the hub. */
    @AutoLogOutput(key = "ShotCalcs/ShootingAtHub")
    public boolean shootingAtHub = false;

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
        shootingAtHub = false;
        rotationOverride = Optional.empty();
        yawError = 0.0;
    }

    private ChassisSpeeds getChassisSpeeds() {
        var currentSpeeds = drive.getFieldSpeeds();
        // In case we are doing shoot-on-the-move in auto
        if (DriverStation.isAutonomous()) return currentSpeeds;
        var desiredSpeeds = desiredSpeedsSupplier.get();
        double weight = SOTM_DESIRED_SPEEDS_WEIGHT.get();
        // For calculating shoot-on-the-move in teleop, we need to know vx, vy, and omega in the future.
        // We estimate this as a weighted average between the desired speeds that the driver controller
        // is commanding, and the current robot speeds. This also fixes stability issues.
        return new ChassisSpeeds(
            desiredSpeeds.vxMetersPerSecond * weight + currentSpeeds.vxMetersPerSecond * (1 - weight),
            desiredSpeeds.vyMetersPerSecond * weight + currentSpeeds.vyMetersPerSecond * (1 - weight),
            desiredSpeeds.omegaRadiansPerSecond * weight + currentSpeeds.omegaRadiansPerSecond * (1 - weight)
        );
    }

    // Commands
    /** Spins up the shooter and aims the robot to the specified target. */
    public Command spinupAndAimCmd(Target target) {
        var cmd = shooter.setVelocityCmd(() -> {
            shotSetpoint = ShotCalcsKt.getShotSetpoint(
                drive.getPose(), getChassisSpeeds(), shooter, true, target
            );
            shootingAtHub = target == Target.HUB;
            rotationOverride = Optional.of(shotSetpoint.yaw());
            yawError = MathUtil.angleModulus(shotSetpoint.yaw().minus(drive.getPose().getRotation()).getRadians());
            return RadiansPerSecond.of(shotSetpoint.radPerSec() * speedAdjustment.getAsDouble());
        })
            .finallyDo(this::resetState);
        return logged(cmd, "SpinupAndAim(" + target + ")");
    }

    public Command shootInAutoCmd(Target target, double agitateDelaySecs) {
        return Commands.parallel(
            spinupAndAimCmd(target),
            CmdSequence.of(
                Commands.waitUntil(this::canSerialize),
                Commands.waitSeconds(0.3),
                serializer.runCmd()
            ),
            CmdSequence.of(
                intake.lowAgitateCmd().withTimeout(agitateDelaySecs),
                intake.highAgitateCmd()
            )
        );
    }

    public Command autoStartCmd(AutoTrajectory traj) {
        return CmdSequence.of(
            traj.resetOdometry(),
            traj.spawnCmd()
        );
    }

    public Command intakeInAutoCmd(double timeUntilSpeedIncrease) {
        return CmdSequence.of(
            intake.deployCmd(1.0, drive::getRobotSpeeds)
                .withTimeout(timeUntilSpeedIncrease),
            intake.deployCmd(1.25, drive::getRobotSpeeds)
        );
    }

    public Command hubShotSpinupCmd(AutoTrajectory traj, double timestamp) {
        var potentialSample = traj.getRawTrajectory().sampleAt(timestamp, false);
        if (potentialSample.isEmpty()) return Commands.none();
        var sample = potentialSample.get();
        var flippedSample = sample.flipped();
        return hubShotSpinupCmd(
            () -> (AllianceColor.isRed() ? flippedSample : sample).getPose(),
            () -> (AllianceColor.isRed() ? flippedSample : sample).getChassisSpeeds()
        );
    }

    public Command hubShotSpinupCmd(AutoTrajectory traj) {
        var zeroSpeed = new ChassisSpeeds();
        return hubShotSpinupCmd(() -> traj.getFinalPose().orElse(drive.getPose()), () -> zeroSpeed);
    }

    public Command hubShotSpinupCmd(
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> speedsSupplier
    ) {
        var cmd = shooter.setVelocityCmd(() -> {
            shotSetpoint = ShotCalcsKt.getShotSetpoint(
                poseSupplier.get(), speedsSupplier.get(),
                shooter, true, Target.HUB
            );
            shootingAtHub = true;
            return RadiansPerSecond.of(shotSetpoint.radPerSec());
        })
            .finallyDo(this::resetState);
        return logged(cmd, "HubShotSpinup");
    }

    public Command manualHubShotCmd() {
        return CmdSequence.of(
            Commands.runOnce(() -> {
                var pose = new Pose2d(1.75, 4.05, Rotation2d.kZero);
                drive.resetPose(AllianceColor.isRed() ? flip(pose) : pose);
            }),
            spinupAndAimCmd(Target.HUB)
        );
    }
}
