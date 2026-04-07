package robot.controllers;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import lib.Tunable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import robot.constants.RobotConfig;
import robot.subsystems.drive.SwerveConfig;
import robot.subsystems.shooter.Shooter;

import java.util.Optional;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kLeftRumble;

@SuppressWarnings("FieldCanBeLocal")
public class DriverController extends CommandPS5Controller {
    private static final Tunable<Double>
        SPEED_REDUCTION = Tunable.of("SpeedReduction", 1),
        AIM_KP = Tunable.of("ShotCalcs/Aiming/KP", 5.5),
        AIM_KD = Tunable.of("ShotCalcs/Aiming/KD", 0.03),
        SWISHING_RATE_MULTIPLIER = Tunable.of("SwishingRateMultiplier", 13.0),
        MAX_LINEAR_SWISH_OUTPUT = Tunable.of("MaxSwishOutput/Linear", 0.07),
        MAX_ANGULAR_SWISH_OUTPUT = Tunable.of("MaxSwishOutput/Angular", 0.03);

    // Linear Filter Equation: Y = C * X + (1 - C) * Y_previous
    // C = e^-(0.02/0.1) = e^(-0.2)
    // X = wanted angular velocity
    // Y = filtered angular velocity
    // Effectively, this filter prevents the robot's target angular velocity from changing too quickly.
    private final LinearFilter rotationFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    // Limits robot acceleration when running shoot on the move.
    private final SlewRateLimiter
        forwardLimiter = new SlewRateLimiter(0.5),
        strafeLimiter = new SlewRateLimiter(0.5);

    private final FieldCentric swerveReq = new FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity);
    private final FieldCentricFacingAngle facingAngleSwerveReq = new FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withCenterOfRotation(Shooter.ROBOT_TO_LAUNCH_POINT.getTranslation());
    private final RobotCentric swishingSwerveReq = new RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity);

    private boolean aimToTargetInit = false;
    @AutoLogOutput private double prevTargetRad = 0.0; // the previous target angle of the robot.
    @AutoLogOutput private double forward = 0, strafe = 0, rotation = 0;

    private final double maxVelMetersPerSec, maxVelRadPerSec;

    public DriverController(int port, SwerveConfig config) {
        super(port);
        this.maxVelMetersPerSec = config.maxVel().in(MetersPerSecond);
        this.maxVelRadPerSec = config.maxAngularVel().in(RadiansPerSecond);
    }

    @AutoLogOutput
    private double swerveSpeedModifier() {
        if (L1().getAsBoolean()) {
            return SPEED_REDUCTION.get() / 3.0;
        }
        double output = getL2Axis();
        Logger.recordOutput("RawL2Axis", output);
        output = MathUtil.applyDeadband(output, 0.1, 1);
        output = (2 - (output / 2.0 + 0.5)) / 2;
        return Math.min(1.0, output * SPEED_REDUCTION.get());
    }

    public SwerveRequest getSwerveRequest() {
        double scalar = swerveSpeedModifier();
        forward = -getLeftY() * scalar;
        strafe = -getLeftX() * scalar;
        rotation = -getRightX() * scalar;
        if (povLeft().getAsBoolean()) {
            strafe = 0.12;
            forward = 0;
        } else if (povRight().getAsBoolean()) {
            strafe = -0.12;
            forward = 0;
        }
        // don't use slew rate limits for normal drive requests
        forwardLimiter.reset(forward);
        strafeLimiter.reset(strafe);
        rotationFilter.reset();
        aimToTargetInit = false;
        return swerveReq
            .withVelocityX(forward * maxVelMetersPerSec)
            .withVelocityY(strafe * maxVelMetersPerSec)
            .withDeadband(0.1 * scalar * maxVelMetersPerSec)
            .withRotationalRate(rotation * maxVelRadPerSec)
            .withRotationalDeadband(0.1 * scalar * maxVelRadPerSec);
    }

    public SwerveRequest getSwerveRequest(Optional<Rotation2d> targetAngle, boolean limitAccel) {
        Logger.recordOutput("DriverController/RawLeftY", -getLeftY());
        Logger.recordOutput("DriverController/RawLeftX", -getLeftX());
        Logger.recordOutput("DriverController/Connected", isConnected());
        if (targetAngle.isEmpty()) return getSwerveRequest();
        double scalar = swerveSpeedModifier();
        forward = -getLeftY() * scalar / 3.0;
        strafe = -getLeftX() * scalar / 3.0;
        if (limitAccel) {
            forward = forwardLimiter.calculate(forward);
            strafe = strafeLimiter.calculate(strafe);
        }
        double deltaRad = MathUtil.angleModulus(targetAngle.get().getRadians() - prevTargetRad);
        double radiansPerSec = aimToTargetInit ? rotationFilter.calculate(deltaRad / 0.02) : 0;
        prevTargetRad = targetAngle.get().getRadians();
        aimToTargetInit = true;
        rotation = radiansPerSec / maxVelRadPerSec;

        return facingAngleSwerveReq
            .withVelocityX(forward * maxVelMetersPerSec)
            .withVelocityY(strafe * maxVelMetersPerSec)
            .withDeadband(0.05 * scalar * maxVelMetersPerSec)
            .withTargetDirection(targetAngle.get())
            .withTargetRateFeedforward(radiansPerSec)
            .withHeadingPID(AIM_KP.get(), 0.0, AIM_KD.get());
    }

    public SwerveRequest getSwishingSwerveRequest() {
        forward = Math.sin(Timer.getTimestamp() * SWISHING_RATE_MULTIPLIER.get()) * MAX_LINEAR_SWISH_OUTPUT.get();
        strafe = Math.sin(Timer.getTimestamp() * SWISHING_RATE_MULTIPLIER.get()) * MAX_ANGULAR_SWISH_OUTPUT.get();
        return swishingSwerveReq
            .withVelocityY(forward * maxVelMetersPerSec)
            .withRotationalRate(strafe * maxVelRadPerSec);
    }

    public Command notifyHubShiftCmd() {
        return Commands.run(() -> setRumble(kLeftRumble, 0.2))
            .withTimeout(0.5)
            .finallyDo(() -> setRumble(kLeftRumble, 0.0))
            .withName("Driver#NotifyHubShift");
    }

    public ChassisSpeeds getDesiredSpeeds() {
        return new ChassisSpeeds(
            forward * maxVelMetersPerSec,
            strafe * maxVelMetersPerSec,
            rotation * maxVelRadPerSec
        );
    }

    // For Rumble to work on PS5 Controllers, we have to run a custom script on the driver station computer.
    // (scripts/driverstation/ps5_controller_rumble.py).
    @Override
    public void setRumble(GenericHID.RumbleType type, double value) {
        var logger = RobotConfig.dashboardLogger;
        int scaledOutput = (int) (MathUtil.clamp(value, 0, 1) * 255);
        switch (type) {
            case kLeftRumble -> logger.log("Rumble/Left", scaledOutput);
            case kRightRumble -> logger.log("Rumble/Right", scaledOutput);
            case kBothRumble -> {
                logger.log("Rumble/Left", scaledOutput);
                logger.log("Rumble/Right", scaledOutput);
            }
        }
    }
}