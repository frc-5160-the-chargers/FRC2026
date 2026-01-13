package robot.subsystems.drive;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.LinearPath;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.RobotMode;
import lib.Tunable;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import robot.vision.Structs.CamPoseEstimate;
import robot.subsystems.ChargerSubsystem;
import robot.subsystems.drive.hardware.MapleSimSwerveHardware;
import robot.subsystems.drive.hardware.SwerveDataAutoLogged;
import robot.subsystems.drive.hardware.SwerveHardware;
import lombok.Getter;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import java.text.DecimalFormat;
import java.util.Optional;
import java.util.function.Supplier;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.units.Units.*;

/**
 * A subsystem that controls the driving of the robot. In each corner of the robot, there is
 * one motor responsible for spinning the wheel, and another for changing the direction of the wheel.
 */
public class SwerveSubsystem extends ChargerSubsystem {
    private final Tunable<Double>
        translationKP = Tunable.of(key("TranslationKP"), 3.5),
        rotationKP = Tunable.of(key("RotationKP"), 8),
        rotationKD = Tunable.of(key("RotationKD"), 0.02),
        alignTolerance = Tunable.of("Alignment/Tolerance", 0.015),
        alignMaxAccel = Tunable.of("Alignment/MaxAccel (m per s^2)", 10),
        alignMaxAngularAccel = Tunable.of("Alignment/MaxAngularAccel (rad per s^2)", 50);

    private final SwerveConfig config;
    @Getter private final SwerveDriveSimulation mapleSim;
    private final SwerveDrivePoseEstimator replayPoseEst;
    private final PIDController
        xPoseController = new PIDController(0, 0, 0.1),
        yPoseController = new PIDController(0, 0, 0.1),
        rotationController = new PIDController(0, 0, 0);
    private boolean replayPoseEstInit = false; // whether the replay pose est has initialized.
    // The control request for path following.
    private final SwerveRequest.ApplyFieldSpeeds pathFollowReq =
        new SwerveRequest.ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity);
    private LinearPath alignment; // The autoalign handler.
    private final SwerveHardware io; // The underlying hardware powering this drivetrain.
    private final SwerveDataAutoLogged inputs = new SwerveDataAutoLogged();

    /** A pose estimate that will be replayed correctly. */
    @Getter private Pose2d pose = Pose2d.kZero;
    @Setter private boolean simulatePoseEstDrift = true;

    public SwerveSubsystem(SwerveConfig config) {
        this.config = config;
        mapleSim = new SwerveDriveSimulation(config.mapleSimConfig(), Pose2d.kZero);
        io = RobotMode.isSim()
            ? MapleSimSwerveHardware.create(config, mapleSim)
            : new SwerveHardware(config);
        replayPoseEst = new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(config.moduleTranslations()),
            Rotation2d.kZero, getModPositions(), Pose2d.kZero,
            config.encoderStdDevs(), VecBuilder.fill(0.6, 0.6, 0.6)
        );
        configureAlignment();
        alignMaxAccel.onChange(this::configureAlignment);
        alignMaxAngularAccel.onChange(this::configureAlignment);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private void configureAlignment() {
        double radius = config.drivebaseRadius().in(Meters);
        var linear = new Constraints(config.maxVel().in(MetersPerSecond), alignMaxAccel.get());
        var angular = new Constraints(linear.maxVelocity / radius, alignMaxAngularAccel.get());
        alignment = new LinearPath(linear, angular);
    }

    private SwerveRequest applyPID(SwerveRequest request) {
        if (request instanceof SwerveRequest.FieldCentricFacingAngle r) {
            r.HeadingController.setPID(rotationKP.get(), 0, rotationKD.get());
        } else if (request instanceof SwerveRequest.RobotCentricFacingAngle r) {
            r.HeadingController.setPID(rotationKP.get(), 0, rotationKD.get());
        }
        return request;
    }

    /**
     * Returns a command that applies the given request repeatedly.
     * Will implicitly inject PID constants into facing angle requests.
     */
    public Command driveCmd(Supplier<SwerveRequest> requestSupplier) {
        String requestName = requestSupplier.get().getClass().getSimpleName();
        return this.run(() -> io.setControl(applyPID(requestSupplier.get())))
            .withName("DriveCmd (" + requestName + ")");
    }

    private SwerveModulePosition[] getModPositions() {
        if (inputs.poseEstFrames.length == 0) {
            return new SwerveModulePosition[]{
                new SwerveModulePosition(), new SwerveModulePosition(),
                new SwerveModulePosition(), new SwerveModulePosition()
            };
        }
        return inputs.poseEstFrames[inputs.poseEstFrames.length - 1].positions();
    }

    private void updatePathFollowReq(ChassisSpeeds goalSpeed, Pose2d goalPose) {
        xPoseController.setP(translationKP.get());
        yPoseController.setP(translationKP.get());
        rotationController.setPID(rotationKP.get(), 0, rotationKD.get());
        var target = ChassisSpeeds.discretize(goalSpeed, 0.02);
        target.vxMetersPerSecond += xPoseController.calculate(pose.getX(), goalPose.getX());
        target.vyMetersPerSecond += yPoseController.calculate(pose.getY(), goalPose.getY());
        target.omegaRadiansPerSecond += rotationController.calculate(
            angleModulus(pose.getRotation().getRadians()),
            angleModulus(goalPose.getRotation().getRadians())
        );
        pathFollowReq.Speeds = target;
    }

    @Override
    public void loggedPeriodic() {
        io.refreshData(inputs);
        // If not in replay mode, logs every value.
        // If in replay mode, overrides every variable with values from the log file.
        Logger.processInputs(getName(), inputs);
        // CTRE's SwerveDrivetrain calculates a pose for us(inputs.notReplayedPose),
        // however, it won't update during replay mode if we change our AprilTag filtering algorithms.
        // So in replay mode, we feed data logged on the real robot into a separate pose estimator
        // that can accept replayed vision measurements.
        if (RobotMode.get() == RobotMode.REPLAY) {
            for (var frame: inputs.poseEstFrames) {
                if (!replayPoseEstInit) {
                    replayPoseEstInit = true;
                    replayPoseEst.resetPosition(
                        frame.heading(), frame.positions(), inputs.notReplayedPose
                    );
                }
                pose = replayPoseEst.updateWithTime(
                    frame.timestampSecs(), frame.heading(), frame.positions()
                );
            }
            Logger.recordOutput(key("ReplayedPose"), pose);
        } else {
            pose = inputs.notReplayedPose;
        }
        if (RobotMode.isSim()) {
            Logger.recordOutput(key("TruePose"), getTruePose());
            if (!simulatePoseEstDrift) pose = getTruePose();
        }
    }

    /** The input data of this drivetrain. */
    public Optional<SwerveDataAutoLogged> getInputs() {
        if (inputs.currentStates.length == 0) return Optional.empty();
        return Optional.of(inputs);
    }

    /** In sim, returns the true pose of the robot without odometry drift. */
    public Pose2d getTruePose() {
        return RobotMode.isSim() ? mapleSim.getSimulatedDriveTrainPose() : pose;
    }

    /**
     * Resets the pose of the robot. The pose should be from the
     * {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
     */
    public void resetPose(Pose2d pose) {
        io.resetNotReplayedPose(pose);
        if (RobotMode.get() != RobotMode.REPLAY) return;
        replayPoseEst.resetPosition(pose.getRotation(), getModPositions(), pose);
    }

    private static class AutoAlignState {
        LinearPath.State setpoint = new LinearPath.State();
        double distToGoal = 0;
    }

    public Command alignCmd(boolean indefinite, Supplier<Pose2d> targetPoseSupplier) {
        var state = new AutoAlignState();
        return this.run(() -> {
            var goal = targetPoseSupplier.get();
            state.setpoint = alignment.calculate(0.02, state.setpoint, goal);
            state.distToGoal = Math.hypot(goal.getX() - pose.getX(), goal.getY() - pose.getY());
            updatePathFollowReq(state.setpoint.speeds, state.setpoint.pose);
            io.setControl(pathFollowReq);
        })
            .beforeStarting(() -> state.setpoint = new LinearPath.State(pose, getFieldSpeeds()))
            .until(() -> !indefinite && state.distToGoal < alignTolerance.get())
            .withName("AutoAlignCmd");
    }

    /** Adds a vision measurement to this drivetrain's pose estimator. */
    public void addVisionMeasurement(CamPoseEstimate estimate) {
        if (RobotMode.get() == RobotMode.REPLAY) {
            double time = estimate.timestampSecs() + inputs.timeOffsetSecs;
            replayPoseEst.addVisionMeasurement(
                estimate.pose(), time, estimate.deviations()
            );
        } else {
            io.addVisionMeasurement(estimate, inputs.timeOffsetSecs);
        }
    }

    /** Fetches the field-relative speeds of this drivetrain. */
    @AutoLogOutput
    public ChassisSpeeds getFieldSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            inputs.robotRelativeSpeeds, pose.getRotation()
        );
    }

    /** Creates an AutoFactory, a utility class for following choreo trajectories. */
    public AutoFactory createAutoFactory() {
        return new AutoFactory(
            () -> pose, this::resetPose, this::followChoreoTraj,
            true, this, this::logTrajectory
        );
    }

    private void followChoreoTraj(SwerveSample trajSample) {
        updatePathFollowReq(trajSample.getChassisSpeeds(), trajSample.getPose());
        pathFollowReq.WheelForceFeedforwardsX = trajSample.moduleForcesX();
        pathFollowReq.WheelForceFeedforwardsY = trajSample.moduleForcesY();
        io.setControl(pathFollowReq);
    }

    private void logTrajectory(Trajectory<SwerveSample> trajectory, boolean isStart) {
        Logger.recordOutput(key("CurrentTraj/Name"), trajectory.name());
        if (RobotMode.get() != RobotMode.REPLAY && DriverStation.isFMSAttached()) {
            return; // don't log trajectory during matches, use replay mode to do so instead
        }
        var samples = trajectory.samples().toArray(new SwerveSample[0]);
        Logger.recordOutput(key("CurrentTraj/Samples"), samples);
    }

    /**
     * A command that applies a set amount of current to the drive motors.
     * Use {@link SwerveRequest.SysIdSwerveTranslation} for the voltage equivalent.
     */
    public Command applyCurrent(double amps) {
        var driveReq = new TorqueCurrentFOC(amps);
        var steerReq = new PositionVoltage(0);
        return this.run(() -> io.setControlCustom(driveReq, steerReq)).withName("Apply Current");
    }

    private static class CharacterizationState {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public Command characterizeWheelRadiusCmd() {
        var limiter = new SlewRateLimiter(0.05);
        var state = new CharacterizationState();
        var req = new SwerveRequest.RobotCentric();
        var movementCmd = Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(() -> limiter.reset(0.0)),
            // Turn in place, accelerating up to full speed
            driveCmd(() -> req.withRotationalRate(limiter.calculate(2)))
        );
        var measurementCmd = Commands.sequence(
            Commands.waitSeconds(2.0),
            Commands.runOnce(() -> {
                state.positions = getModPositions();
                state.gyroDelta = 0.0;
                state.lastAngle = pose.getRotation();
            }),
            Commands.run(() -> {
                state.gyroDelta += Math.abs(pose.getRotation().minus(state.lastAngle).getRadians());
                state.lastAngle = pose.getRotation();
            })
        ).finallyDo(() -> {
            var currPositions = getModPositions();
            double wheelDeltaM = 0.0;
            for (int i = 0; i < 4; i++) {
                wheelDeltaM += Math.abs(
                    currPositions[i].distanceMeters - state.positions[i].distanceMeters
                ) / 4.0;
            }
            double wheelDeltaRad = wheelDeltaM / config.moduleConsts()[0].WheelRadius;
            double wheelRadius = (state.gyroDelta * config.drivebaseRadius().in(Meters)) / wheelDeltaRad;
            double wheelRadiusIn = Meters.of(wheelRadius).in(Inches);
            var formatter = new DecimalFormat("#0.000000000000000000000000000");
            System.out.println("********** Wheel Radius Characterization Results **********");
            System.out.println("\tWheel Delta: " + formatter.format(wheelDeltaM) + " rotations");
            System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " rotations");
            System.out.println("\tWheel Radius: " + formatter.format(wheelRadiusIn) + " inches");
        });
        return Commands.parallel(movementCmd, measurementCmd).withName("Wheel Radius Characterization");
    }
}