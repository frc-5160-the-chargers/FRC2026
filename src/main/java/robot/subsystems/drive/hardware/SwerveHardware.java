package robot.subsystems.drive.hardware;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lib.RobotMode;
import lib.Tunable;
import robot.constants.GlobalConsts;
import robot.subsystems.drive.SwerveConfig;
import robot.vision.Structs.CamPoseEstimate;
import robot.subsystems.drive.hardware.SwerveData.OdometryFrame;

import java.util.*;

/** A class that wraps CTRE's {@link SwerveDrivetrain} with replay support. */
public class SwerveHardware {
    protected final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    private final SwerveDebugging debugging;
    private final Queue<OdometryFrame> poseEstBuffer = new ArrayDeque<>();
    private SwerveDrivetrain.SwerveDriveState latest;

    public SwerveHardware(SwerveConfig config) {
        drivetrain = new SwerveDrivetrain<>(
            TalonFX::new, TalonFX::new, CANcoder::new,
            config.driveConsts(), config.moduleConsts()
        );
        debugging = new SwerveDebugging(drivetrain);
        latest = drivetrain.getStateCopy();
        if (RobotMode.get() == RobotMode.REPLAY) drivetrain.getOdometryThread().stop();
        // registerTelemetry() technically means "register a function that logs data",
        // but here we abuse it for data-gathering purposes.
        drivetrain.registerTelemetry(this::recordData);
        drivetrain.setStateStdDevs(config.encoderStdDevs());
        initDashboardTuning(config);
    }

    private synchronized void recordData(SwerveDrivetrain.SwerveDriveState state) {
        latest = state;
        if (debugging.isOverflowing(poseEstBuffer.size())) return;
        // phoenix 6 measures time differently, so we use currentTimeToFPGATime() to correct the timestamp.
        var latestFrame = new OdometryFrame(
            state.RawHeading, state.Timestamp,
            state.ModulePositions[0], state.ModulePositions[1],
            state.ModulePositions[2], state.ModulePositions[3]
        );
        poseEstBuffer.add(latestFrame);
    }

    /** Updates a {@link SwerveDataAutoLogged} instance with the latest data. */
    public void refreshData(SwerveDataAutoLogged inputs) {
        drivetrain.setOperatorPerspectiveForward(
            GlobalConsts.redAlliance() ? Rotation2d.k180deg : Rotation2d.kZero
        );
        debugging.logData();
        inputs.timeOffsetSecs = Utils.fpgaToCurrentTime(0);
        synchronized (this) {
            inputs.poseEstFrames = poseEstBuffer.toArray(new OdometryFrame[0]);
            poseEstBuffer.clear();
            inputs.currentStates = latest.ModuleStates;
            inputs.desiredStates = latest.ModuleTargets;
            inputs.notReplayedPose = latest.Pose;
            inputs.robotRelativeSpeeds = latest.Speeds;
        }
    }

    /** Applies the specified control request to this swerve drivetrain. */
    public void setControl(SwerveRequest request) {
        drivetrain.setControl(request);
    }

    /** Resets the non-replayed pose. */
    public void resetNotReplayedPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    /** Adds a vision measurement to the non-replayed pose estimator. */
    public void addVisionMeasurement(CamPoseEstimate estimate, double timeOffsetSecs) {
        double time = estimate.timestampSecs() + timeOffsetSecs;
        drivetrain.addVisionMeasurement(estimate.pose(), time, estimate.deviations());
    }

    /** Configures neutral mode(brake or coast) on this drivetrain. */
    public void setCoastMode(boolean enabled) {
        drivetrain.configNeutralMode(enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake, 0.015);
    }

    /** Applies custom control requests to the drive and steer motors. Debug only. */
    public void setControlCustom(ControlRequest drive, ControlRequest steer) {
        setControl(new SwerveRequest.Idle());
        for (var module: drivetrain.getModules()) {
            module.getDriveMotor().setControl(drive);
            module.getSteerMotor().setControl(steer);
        }
    }

    private void initDashboardTuning(SwerveConfig config) {
        var driveGains = config.moduleConsts()[0].DriveMotorGains;
        var steerGains = config.moduleConsts()[0].SteerMotorGains;
        Tunable.of("SwerveSubsystem/DriveMotor/KP", driveGains.kP)
            .onChange(kP -> applyDriveGains(driveGains.withKP(kP)));
        Tunable.of("SwerveSubsystem/SteerMotor/KP", steerGains.kP)
            .onChange(kP -> applySteerGains(steerGains.withKP(kP)));
        Tunable.of("SwerveSubsystem/SteerMotor/KD", steerGains.kD)
            .onChange(kD -> applySteerGains(steerGains.withKD(kD)));
        Tunable.of("SwerveSubsystem/CoastMode", false)
            .onChange(this::setCoastMode);
    }

    private void applySteerGains(Slot0Configs configs) {
        for (var module: drivetrain.getModules()) {
            module.getSteerMotor().getConfigurator().apply(configs, 0.015);
        }
    }

    private void applyDriveGains(Slot0Configs configs) {
        for (var module: drivetrain.getModules()) {
            module.getDriveMotor().getConfigurator().apply(configs, 0.015);
        }
    }
}