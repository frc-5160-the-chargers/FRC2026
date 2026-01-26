package robot.subsystems.drive;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.utility.LinearPath;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.RobotMode;
import lib.Tunable;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.ChargerSubsystem;
import robot.subsystems.drive.hardware.MapleSimSwerveHardware;
import robot.subsystems.drive.hardware.SwerveDataAutoLogged;
import robot.subsystems.drive.hardware.SwerveHardware;
import robot.vision.DataTypes.CamPoseEstimate;

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
        alignKP = Tunable.of(key("AutoAlign/TranslationKP"), 2),
        choreoKP = Tunable.of(key("Choreo/TranslationKP"), 3.5),
        rotationKP = Tunable.of(key("AutoAlign/RotationKP"), 2),
        alignTolerance = Tunable.of(key("AutoAlign/Tolerance"), 0.015),
        alignMaxAccel = Tunable.of(key("AutoAlign/MaxAccel (m per s^2)"), 10),
        alignMaxAngularAccel = Tunable.of(key("AutoAlign/MaxAngularAccel (rad per s^2)"), 40),
        wrcMaxSpeed = Tunable.of(key("WheelRadiusCharacterization/MaxSpeed(rad per s)"), 2.0);

    private final SwerveConfig config;
    private final SwerveDrivePoseEstimator replayPoseEst;
    private final PIDController
        xPoseController = new PIDController(0, 0, 0),
        yPoseController = new PIDController(0, 0, 0),
        rotationController = new PIDController(0, 0, 0);
    private final ApplyFieldSpeeds pathFollowReq =
        new ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity);
    private boolean poseEstInitialized = false;
    private LinearPath alignment;
    private final SwerveHardware io; // The underlying hardware powering this drivetrain.
    private final SwerveDataAutoLogged inputs = new SwerveDataAutoLogged();

    /** A pose estimate that will be replayed correctly. */
    @Getter private Pose2d pose = Pose2d.kZero;

    public SwerveSubsystem(SwerveConfig config) {
        super(config.name());
        this.config = config;
        io = RobotMode.isSim()
            ? MapleSimSwerveHardware.create(config)
            : new SwerveHardware(config);
        replayPoseEst = new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(config.moduleTranslations()),
            Rotation2d.kZero, getModPositions(), Pose2d.kZero
        );
        // Configure Auto-Align utilities
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

    /**
     * Returns a command that applies the given request repeatedly.
     * Will implicitly inject PID constants into the {@link FieldCentricFacingAngle} request.
     */
    public Command driveCmd(Supplier<SwerveRequest> requestSupplier) {
        return this.run(() -> {
            var request = requestSupplier.get();
            if (request instanceof FieldCentricFacingAngle r && r.HeadingController.getP() == 0) {
                request = r.withHeadingPID(rotationKP.get(), 0, 0);
            }
            io.setControl(request);
            Logger.recordOutput(key("Request"), request.getClass().getSimpleName());
        })
            .withName("SwerveDriveCmd");
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

    private void updatePathFollowReq(ChassisSpeeds goalVel, Pose2d goalPose, double transKP) {
        xPoseController.setP(transKP);
        yPoseController.setP(transKP);
        rotationController.setP(rotationKP.get());
        goalVel.vxMetersPerSecond += xPoseController.calculate(pose.getX(), goalPose.getX());
        goalVel.vyMetersPerSecond += yPoseController.calculate(pose.getY(), goalPose.getY());
        goalVel.omegaRadiansPerSecond += rotationController.calculate(
            angleModulus(pose.getRotation().getRadians()),
            angleModulus(goalPose.getRotation().getRadians())
        );
        pathFollowReq.Speeds = goalVel;
    }

    @Override
    public void loggedPeriodic() {
        io.refreshData(inputs);
        // If not in replay mode, logs every value.
        // If in replay mode, overrides every variable with values from the log file.
        Logger.processInputs(getName(), inputs);
        // Syncs the replay pose estimator with the robot's state.
        if (inputs.bufferOverflow) return;
        if (!poseEstInitialized) {
            poseEstInitialized = true;
            var fm = inputs.poseEstFrames[inputs.poseEstFrames.length - 1];
            replayPoseEst.resetPosition(fm.heading(), fm.positions(), inputs.pose);
        }
        // The value of inputs.pose is pre-computed by CTRE,
        // but won't update during replay mode if vision algorithms are changed.
        // So in replay mode, a separate pose estimator is used and updated.
        if (RobotMode.get() == RobotMode.REPLAY) {
            for (var frame: inputs.poseEstFrames) {
                pose = replayPoseEst.updateWithTime(
                    frame.timestampSecs(), frame.heading(), frame.positions()
                );
            }
            Logger.recordOutput(key("ReplayedPose"), pose);
        } else {
            pose = inputs.pose;
        }
    }

    /** The input data of this drivetrain. */
    public Optional<SwerveDataAutoLogged> getInputs() {
        if (inputs.currentStates.length == 0) return Optional.empty();
        return Optional.of(inputs);
    }

    /**
     * Resets the pose of the robot. The pose should be from the
     * {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
     */
    public void resetPose(Pose2d pose) {
        if (poseEstInitialized) {
            io.resetNotReplayedPose(pose);
            if (RobotMode.get() == RobotMode.REPLAY) replayPoseEst.resetPose(pose);
        } else {
            var cmd = Commands.waitUntil(() -> poseEstInitialized)
                .andThen(() -> resetPose(pose))
                .ignoringDisable(true)
                .withName("Delayed resetPose()");
            CommandScheduler.getInstance().schedule(cmd);
        }
    }

    /** Resets only the heading of the robot. */
    public void resetHeading(Rotation2d heading) {
        resetPose(new Pose2d(pose.getX(), pose.getY(), heading));
    }

    private static class AutoAlignState {
        LinearPath.State setpoint = new LinearPath.State();
        double distToGoal = 0;
    }

    /** Returns a command that aligns the robot to a specified pose. */
    public Command alignCmd(Supplier<Pose2d> targetPoseSupplier) {
        var state = new AutoAlignState();
        return this.run(() -> {
            var goal = targetPoseSupplier.get();
            state.setpoint = alignment.calculate(0.02, state.setpoint, goal);
            state.distToGoal = Math.hypot(goal.getX() - pose.getX(), goal.getY() - pose.getY());
            updatePathFollowReq(state.setpoint.speeds, state.setpoint.pose, alignKP.get());
            io.setControl(pathFollowReq);
            Logger.recordOutput(key("Request"), "AutoAlign");
        })
            .until(() -> state.distToGoal < alignTolerance.get())
            .beforeStarting(() -> state.setpoint = new LinearPath.State(pose, getFieldSpeeds()))
            .finallyDo(() -> io.setControl(new SwerveRequest.SwerveDriveBrake()))
            .withName("AutoAlignCmd");
    }

    /** Adds a vision measurement to this drivetrain's pose estimator. */
    public void addVisionMeasurement(CamPoseEstimate estimate) {
        if (RobotMode.get() == RobotMode.REPLAY) {
            double time = estimate.timestampSecs() + inputs.timeOffsetSecs;
            replayPoseEst.addVisionMeasurement(
                estimate.pose(), time, estimate.deviations()
            );
        }
        io.addVisionMeasurement(estimate, inputs.timeOffsetSecs);
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

    private void followChoreoTraj(SwerveSample target) {
        updatePathFollowReq(target.getChassisSpeeds(), target.getPose(), choreoKP.get());
        pathFollowReq.WheelForceFeedforwardsX = target.moduleForcesX();
        pathFollowReq.WheelForceFeedforwardsY = target.moduleForcesY();
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
            driveCmd(() -> req.withRotationalRate(limiter.calculate(wrcMaxSpeed.get())))
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
                wheelDeltaM += Math.abs(currPositions[i].distanceMeters - state.positions[i].distanceMeters);
            }
            wheelDeltaM /= 4.0;
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