package robot.subsystems.drive.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lib.RobotMode;
import lib.Tunable;
import lib.hardware.MotorStats;
import lib.hardware.SignalRefresh;
import org.littletonrobotics.junction.Logger;
import robot.SharedData;
import robot.subsystems.drive.SwerveConfig;
import robot.vision.DataTypes.CamPoseEstimate;
import robot.subsystems.drive.hardware.SwerveData.OdometryFrame;

import java.util.*;

/** A class that wraps CTRE's {@link SwerveDrivetrain} with replay support. */
public class SwerveHardware {
    private static final int MAX_BUFFER_CAPACITY = 60;
    protected final String name;
    protected final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    private final BaseStatusSignal[] debugSignals = new BaseStatusSignal[5];
    private final Queue<OdometryFrame> poseEstBuffer = new ArrayDeque<>();
    private SwerveDrivetrain.SwerveDriveState latest;

    public SwerveHardware(SwerveConfig config) {
        drivetrain = new SwerveDrivetrain<>(
            TalonFX::new, TalonFX::new, CANcoder::new,
            config.driveConsts(), config.moduleConsts()
        );
        name = config.name() + "/";
        latest = drivetrain.getStateCopy();
        if (RobotMode.get() == RobotMode.REPLAY) drivetrain.getOdometryThread().stop();
        // registerTelemetry() technically means "register a function that logs data",
        // but here we abuse it for data-gathering purposes.
        drivetrain.registerTelemetry(this::recordData);
        initDashboardTuning(config);
        initDebugSignals();
    }

    private synchronized void recordData(SwerveDrivetrain.SwerveDriveState state) {
        latest = state;
        if (poseEstBuffer.size() > MAX_BUFFER_CAPACITY) return;
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
            SharedData.redAlliance() ? Rotation2d.k180deg : Rotation2d.kZero
        );
        inputs.timeOffsetSecs = Utils.fpgaToCurrentTime(0);
        synchronized (this) {
            inputs.bufferOverflow = poseEstBuffer.size() > MAX_BUFFER_CAPACITY;
            inputs.poseEstFrames = poseEstBuffer.toArray(new OdometryFrame[0]);
            poseEstBuffer.clear();
            inputs.currentStates = latest.ModuleStates;
            inputs.desiredStates = latest.ModuleTargets;
            inputs.pose = latest.Pose;
            inputs.robotRelativeSpeeds = latest.Speeds;
        }
        if (RobotMode.get() != RobotMode.REPLAY) logDebugData();
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

    private void initDashboardTuning(SwerveConfig config) {
        // Front left, front right, back left, back right
        // int[], int... - arrays!
        var driveGains = config.moduleConsts()[0].DriveMotorGains;
        var steerGainsFront = config.moduleConsts()[0].SteerMotorGains;
        var steerGainsBack = config.moduleConsts()[2].SteerMotorGains;
        Tunable.of(name + "DriveMotor/KP", driveGains.kP)
            .onChange(kP -> applyDriveGains(driveGains.withKP(kP)));
        Tunable.of(name + "SteerMotor/KP/front", steerGainsFront.kP)
            .onChange(kP -> applySteerGains(steerGainsFront.withKP(kP)));
        Tunable.of(name + "SteerMotor/KD/front", steerGainsFront.kD)
            .onChange(kD -> applySteerGains(steerGainsFront.withKD(kD)));
        Tunable.of(name + "SteerMotor/KP/back", steerGainsBack.kP)
                .onChange(kP -> applySteerGains(steerGainsBack.withKP(kP)));
        Tunable.of(name + "SteerMotor/KD/back", steerGainsBack.kD)
                .onChange(kD -> applySteerGains(steerGainsBack.withKD(kD)));
        Tunable.of(name + "CoastMode", false)
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

    private void initDebugSignals() {
        for (int i = 0; i < 4; i++) {
            debugSignals[i] = drivetrain.getModule(i).getEncoder().getVersion();
        }
        debugSignals[4] = drivetrain.getPigeon2().getVersion();
        SignalRefresh.register(10, drivetrain.getPigeon2().getNetwork(), debugSignals);
    }

    private void logDebugData() {
        var driveStats = new MotorStats[4];
        var steerStats = new MotorStats[4];
        var encodersConnected = new boolean[4];
        for (int i = 0; i < 4; i++) {
            driveStats[i] = MotorStats.from(drivetrain.getModule(i).getDriveMotor());
            steerStats[i] = MotorStats.from(drivetrain.getModule(i).getSteerMotor());
            encodersConnected[i] = debugSignals[i].getStatus().isOK();
        }
        Logger.recordOutput(name + "DriveMotorData", driveStats);
        Logger.recordOutput(name + "SteerMotorData", steerStats);
        Logger.recordOutput(name + "EncodersConnected", encodersConnected);
        Logger.recordOutput(name + "GyroConnected", debugSignals[4].getStatus().isOK());
    }
}