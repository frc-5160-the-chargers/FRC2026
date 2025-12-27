package robot.subsystems.drive.hardware;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lib.CurrAlliance;
import lib.RobotMode;
import robot.subsystems.drive.SwerveConfig;
import robot.vision.Structs.CamPoseEstimate;
import robot.subsystems.drive.hardware.SwerveData.OdometryFrame;

import java.util.ArrayDeque;
import java.util.Queue;
import java.util.concurrent.locks.ReentrantLock;

/** A class that wraps CTRE's {@link SwerveDrivetrain} with replay support. */
public class SwerveHardware {
    protected final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    private final ReentrantLock stateLock = new ReentrantLock(); // prevents simultaneous modification of poseEstBuffer
    private final Queue<OdometryFrame> poseEstBuffer = new ArrayDeque<>();
    private SwerveDrivetrain.SwerveDriveState latest;

    public SwerveHardware(SwerveConfig config) {
        drivetrain = new SwerveDrivetrain<>(
            TalonFX::new, TalonFX::new, CANcoder::new,
            config.driveConsts(), config.moduleConsts()
        );
        latest = drivetrain.getStateCopy();
        if (RobotMode.get() == RobotMode.REPLAY) drivetrain.getOdometryThread().stop();
        // registerTelemetry() technically means "register a function that logs data",
        // but here we abuse it for data-gathering purposes.
        drivetrain.registerTelemetry(state -> {
            try {
                stateLock.lock();
                latest = state;
                if (poseEstBuffer.size() > 30) return;
                // phoenix 6 measures time differently, so we use currentTimeToFPGATime() to correct the timestamp.
                var latestFrame = new OdometryFrame(
                    state.RawHeading, Utils.currentTimeToFPGATime(state.Timestamp),
                    state.ModulePositions[0], state.ModulePositions[1],
                    state.ModulePositions[2], state.ModulePositions[3]
                );
                poseEstBuffer.add(latestFrame);
            } finally {
                stateLock.unlock();
            }
        });
        drivetrain.setStateStdDevs(config.encoderStdDevs());
    }

    /** Updates a {@link SwerveDataAutoLogged} instance with the latest data. */
    public void refreshData(SwerveDataAutoLogged inputs) {
        drivetrain.setOperatorPerspectiveForward(
            CurrAlliance.red() ? Rotation2d.k180deg : Rotation2d.kZero
        );
        inputs.poseEstValid = drivetrain.isOdometryValid();
        try {
            stateLock.lock();
            inputs.poseEstFrames = poseEstBuffer.toArray(new OdometryFrame[0]);
            poseEstBuffer.clear();
            inputs.currentStates = latest.ModuleStates;
            inputs.desiredStates = latest.ModuleTargets;
            inputs.notReplayedPose = latest.Pose;
            inputs.robotRelativeSpeeds = latest.Speeds;
        } finally {
            stateLock.unlock();
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
    public void addVisionMeasurement(CamPoseEstimate estimate) {
        drivetrain.addVisionMeasurement(
            estimate.pose(),
            // phoenix 6 measures time differently, so we use fpgaToCurrentTime() to correct the timestamp.
            Utils.fpgaToCurrentTime(estimate.timestampSecs()),
            estimate.deviations()
        );
    }

    /** Moves the drivetrain straight at the specified output(amps or volts). */
    public void runDriveMotors(double output) {
        var steerReq = new PositionVoltage(0);
        for (var module: drivetrain.getModules()) {
            var driveReq = switch (module.getDriveClosedLoopOutputType()) {
                case Voltage -> new VoltageOut(output);
                case TorqueCurrentFOC -> new TorqueCurrentFOC(output);
            };
            module.apply(driveReq, steerReq);
        }
    }
}