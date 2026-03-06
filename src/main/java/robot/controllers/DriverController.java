package robot.controllers;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import lib.Tunable;
import org.littletonrobotics.junction.AutoLogOutput;
import robot.constants.RobotConfig;
import robot.subsystems.drive.SwerveConfig;
import robot.subsystems.shooter.Shooter;

import java.util.Optional;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

/**
 * Handles inputs from the driver controller.
 */
@SuppressWarnings("FieldCanBeLocal")
public class DriverController extends CommandPS5Controller {
    private static final Tunable<Double>
        SPEED_REDUCTION = Tunable.of("SpeedReduction", 1),
        HUB_AIM_KP = Tunable.of("HubAiming/KP", 6.3),
        HUB_AIM_KD = Tunable.of("HubAiming/KD", 0.05);

    // Equation: X = C * x_current + (1 - C) * x_previous, C = e^-(0.02/0.1) = e^(-0.2).
    // Effectively, this filter prevents the robot's target angular velocity from oscillating.
    private final LinearFilter rotationFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    // A SlewRateLimiter limits the acceleration of the input.
    private final SlewRateLimiter
        forwardLimiter = new SlewRateLimiter(2.3, -4.0, 0.0),
        strafeLimiter = new SlewRateLimiter(2.3, -4.0, 0.0);

    // The swerve request this controller calculates.
    private final FieldCentric swerveReq = new FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity);
    private final FieldCentricFacingAngle facingAngleSwerveReq = new FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.Velocity);

    private double prevTargetRad = 0.0; // the previous target angle of the robot.
    private final double maxVelMetersPerSec, maxVelRadPerSec;

    // controller axis outputs; range from 0-1.
    @AutoLogOutput private double forward = 0, strafe = 0, rotation = 0;

    public DriverController(int port, SwerveConfig config) {
        super(port);
        this.maxVelMetersPerSec = config.maxVel().in(MetersPerSecond);
        this.maxVelRadPerSec = config.maxAngularVel().in(RadiansPerSecond);
    }

    @AutoLogOutput
    private double swerveSpeedModifier() {
        double output = getL2Axis();
        output = MathUtil.applyDeadband(output, 0.2, 1);
        output = (2 - output) / 2;
        return output * SPEED_REDUCTION.get();
    }

    public SwerveRequest getSwerveRequest() {
        double scalar = swerveSpeedModifier();
        forward = -getLeftY() * scalar;
        strafe = -getLeftX() * scalar;
        rotation = -getRightX() * scalar;
        // don't use slew rate limits for normal drive requests
        forwardLimiter.reset(forward);
        strafeLimiter.reset(strafe);
        return swerveReq
            .withVelocityX(forward * maxVelMetersPerSec)
            .withVelocityY(strafe * maxVelMetersPerSec)
            .withDeadband(0.1 * scalar * maxVelMetersPerSec)
            .withRotationalRate(rotation * maxVelRadPerSec)
            .withRotationalDeadband(0.1 * scalar * maxVelRadPerSec);
    }

    public SwerveRequest getSwerveRequest(
        Optional<Rotation2d> heading,
        Optional<Shooter.Target> shotTarget
    ) {
        if (heading.isEmpty()) return getSwerveRequest();
        double scalar = swerveSpeedModifier();
        forward = -getLeftY() * scalar;
        strafe = -getLeftX() * scalar;
        if (shotTarget.isPresent() && shotTarget.get() == Shooter.Target.HUB) {
            // add slew rate limiting if shooting in hub
            forward = forwardLimiter.calculate(forward);
            strafe = strafeLimiter.calculate(strafe);
        }
        double radiansPerSec = (heading.get().getRadians() - prevTargetRad) / 0.02;
        prevTargetRad = heading.get().getRadians();

        return facingAngleSwerveReq
            .withVelocityX(forward * maxVelMetersPerSec / 2.0)
            .withVelocityY(strafe * maxVelMetersPerSec / 2.0)
            .withDeadband(0.05 * scalar * maxVelMetersPerSec)
            .withTargetDirection(heading.get())
            .withTargetRateFeedforward(rotationFilter.calculate(radiansPerSec))
            .withHeadingPID(HUB_AIM_KP.get(), 0, HUB_AIM_KD.get());
    }

    public Command rumbleCmd(GenericHID.RumbleType type, double value) {
        return Commands.run(() -> setRumble(type, value))
            .finallyDo(() -> setRumble(GenericHID.RumbleType.kBothRumble, 0))
            .withName("DriverControllerRumble");
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