package robot.subsystems.drive.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
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
import lib.AllianceColor;
import lib.RobotMode;
import lib.Tunable;
import lib.hardware.MotorStats;
import lib.hardware.SignalRefresh;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.drive.SwerveConfig;
import robot.subsystems.drive.hardware.SwerveData.OdometryFrame;
import robot.vision.DataTypes.CamPoseEstimate;

import java.util.ArrayDeque;
import java.util.Queue;

/** A class that wraps CTRE's {@link SwerveDrivetrain} with replay support. */
public class SwerveHardware {
    private static final int MAX_BUFFER_CAPACITY = 60;
    protected final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    protected final String name;
    private int cachedDrivetrainId;
    private SwerveJNI dataLogger;
    private final BaseStatusSignal pitchDeg, accelX, accelY;
    private final BaseStatusSignal[] debugSignals = new BaseStatusSignal[5];
    private final Queue<OdometryFrame> poseEstBuffer = new ArrayDeque<>();

    private double xTest, yTest;
    private boolean velocityTestInit = false;

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
        pitchDeg = drivetrain.getPigeon2().getPitch();
        accelX = drivetrain.getPigeon2().getAccelerationX();
        accelY = drivetrain.getPigeon2().getAccelerationY();
        SignalRefresh.register(100.0, drivetrain.getPigeon2().getNetwork(), pitchDeg, accelX, accelY);
        // Makes the addPoseEstFrame() method execute every 0.004 secs(or 250hz).
        dataLogger.JNI_RegisterTelemetry(cachedDrivetrainId, this::addPoseEstFrame);
//        drivetrain.getOdometryThread().setThreadPriority(2);
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

    /** Configures neutral mode(brake or coast) on this drivetrain with an async delay. */
    public void setCoastMode(boolean enabled) {
        var target = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        new Thread(() -> drivetrain.configNeutralMode(target)).start();
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
            AllianceColor.isRed() ? Rotation2d.k180deg : Rotation2d.kZero
        );
        inputs.timeOffsetSecs = Utils.fpgaToCurrentTime(0);
        inputs.pitchDeg = pitchDeg.getValueAsDouble();
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
        if (!velocityTestInit) {
            xTest = inputs.pose.getX();
            yTest = inputs.pose.getY();
            velocityTestInit = true;
        } else {
            xTest += accelX.getValueAsDouble() * 0.5 * 0.02 * 0.02;
            yTest += accelY.getValueAsDouble() * 0.5 * 0.02 * 0.02;
        }
        Logger.recordOutput("PoseTest", new Pose2d(xTest, yTest, Rotation2d.kZero));
        if (RobotMode.get() != RobotMode.REPLAY) logDebugData();
    }

    private SwerveModuleState convertState(SwerveJNI.ModuleState state) {
        return new SwerveModuleState(state.speed, Rotation2d.fromRadians(state.angle));
    }

    private SwerveModulePosition convertPos(SwerveJNI.ModulePosition pos) {
        return new SwerveModulePosition(pos.distance, Rotation2d.fromRadians(pos.angle));
    }

    private void initDashboardTuning(SwerveConfig config) {
        // Front left, front right, back left, back right
        var driveGains = config.moduleConsts()[0].DriveMotorGains;
        var steerGainsF = config.moduleConsts()[0].SteerMotorGains;
        var steerGainsB = config.moduleConsts()[2].SteerMotorGains;
        Tunable.of(name + "DriveMotor/KP", driveGains.kP).onChange(kP -> {
            for (int i = 0; i < 4; i++) {
                var motor = drivetrain.getModule(i).getDriveMotor();
                motor.getConfigurator().apply(driveGains.withKP(kP));
            }
        });
        Tunable.of(name + "FrontSteerMotor/KP", steerGainsF.kP).onChange(kP -> {
            getSteerConfigurator(0).apply(steerGainsF.withKP(kP));
            getSteerConfigurator(1).apply(steerGainsF.withKP(kP));
        });
        Tunable.of(name + "BackSteerMotor/KP", steerGainsB.kP).onChange(kP -> {
            getSteerConfigurator(2).apply(steerGainsB.withKP(kP));
            getSteerConfigurator(3).apply(steerGainsB.withKP(kP));
        });
        Tunable.of(name + "CoastMode", false).onChange(this::setCoastMode);
    }

    private TalonFXConfigurator getSteerConfigurator(int moduleIndex) {
        return drivetrain.getModule(moduleIndex).getSteerMotor().getConfigurator();
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