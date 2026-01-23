package robot.subsystems.drive.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    protected final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    protected final String name;
    private int cachedDrivetrainId;
    private SwerveJNI dataLogger;
    private final BaseStatusSignal[] debugSignals = new BaseStatusSignal[5];
    private final Queue<OdometryFrame> poseEstBuffer = new ArrayDeque<>();

    public SwerveHardware(SwerveConfig config) {
        name = config.name() + "/";
        drivetrain = new SwerveDrivetrain<>(
            TalonFX::new, TalonFX::new, CANcoder::new,
            config.driveConsts(), config.moduleConsts()
        ) {{
            // m_drivetrainId and m_jni are protected variables of SwerveDrivetrain,
            // while cachedDrivetrainId and dataLogger are variables of SwerveHardware.
            cachedDrivetrainId = m_drivetrainId;
            dataLogger = m_jni.clone();
        }};
        // Makes the addPoseEstFrame() method execute every 0.004 secs(or 250hz).
        dataLogger.JNI_RegisterTelemetry(cachedDrivetrainId, this::addPoseEstFrame);
        initDashboardTuning(config);
        initDebugSignals();
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

    // Fetches pose estimation data from the latest drivetrain state,
    // and converts it into a loggable format (the OdometryFrame object).
    private synchronized void addPoseEstFrame() {
        if (poseEstBuffer.size() > MAX_BUFFER_CAPACITY) return;
        var state = dataLogger.driveState;
        var loggableState = new OdometryFrame(
            state.RawHeading, state.Timestamp,
            convertPos(state.ModulePositions[0]), convertPos(state.ModulePositions[1]),
            convertPos(state.ModulePositions[2]), convertPos(state.ModulePositions[3])
        );
        poseEstBuffer.add(loggableState);
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
            var state = dataLogger.driveState;
            for (int i = 0; i < 4; i++) {
                inputs.currentStates[i] = convertState(state.ModuleStates[i]);
                inputs.desiredStates[i] = convertState(state.ModuleTargets[i]);
            }
            inputs.pose = new Pose2d(state.PoseX, state.PoseY, Rotation2d.fromRadians(state.PoseTheta));
            inputs.robotRelativeSpeeds = new ChassisSpeeds(state.SpeedsVx, state.SpeedsVy, state.SpeedsOmega);
        }
        if (RobotMode.get() != RobotMode.REPLAY) logDebugData();
    }

    private SwerveModuleState convertState(SwerveJNI.ModuleState state) {
        return new SwerveModuleState(state.speed, Rotation2d.fromRadians(state.angle));
    }

    private SwerveModulePosition convertPos(SwerveJNI.ModulePosition pos) {
        return new SwerveModulePosition(pos.distance, Rotation2d.fromRadians(pos.angle));
    }

    private void initDashboardTuning(SwerveConfig config) {
        var driveGains = config.moduleConsts()[0].DriveMotorGains;
        var steerGains = config.moduleConsts()[0].SteerMotorGains;
        Tunable.of(name + "DriveMotor/KP", driveGains.kP)
            .onChange(kP -> applyDriveGains(driveGains.withKP(kP)));
        Tunable.of(name + "SteerMotor/KP", steerGains.kP)
            .onChange(kP -> applySteerGains(steerGains.withKP(kP)));
        Tunable.of(name + "SteerMotor/KD", steerGains.kD)
            .onChange(kD -> applySteerGains(steerGains.withKD(kD)));
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