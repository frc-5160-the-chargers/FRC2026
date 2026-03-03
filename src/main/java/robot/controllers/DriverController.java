package robot.controllers;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import lib.Tunable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import robot.subsystems.drive.SwerveConfig;

import java.util.Optional;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

@SuppressWarnings("FieldCanBeLocal")
public class DriverController extends CommandPS5Controller implements Subsystem {
    private static final Tunable<Double>
        SPEED_REDUCTION = Tunable.of("SpeedReduction", 1),
        HUB_AIM_KP = Tunable.of("HubAiming/KP", 5.5),
        HUB_AIM_KD = Tunable.of("HubAiming/KD", 0.03);

    private final LoggedNetworkNumber
        leftRumble = new LoggedNetworkNumber("Rumble/Left", 0),
        rightRumble = new LoggedNetworkNumber("Rumble/Right", 0);
    private final PIDController headingPID = new PIDController(HUB_AIM_KP.get(), 0, HUB_AIM_KD.get());
    private final LinearFilter headingFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private final SlewRateLimiter
        forwardLimiter = new SlewRateLimiter(1.3, -3.0, 0.0),
        strafeLimiter = new SlewRateLimiter(1.3, -3.0, 0.0);

    private final FieldCentric swerveReq = new FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity);
    private double prevTargetRad = 0.0;
    private final double maxVelMetersPerSec, maxVelRadPerSec;
    @AutoLogOutput private double forward = 0, strafe = 0, rotation = 0;

    public DriverController(int port, SwerveConfig config) {
        super(port);
        register();
        this.maxVelMetersPerSec = config.maxVel().in(MetersPerSecond);
        this.maxVelRadPerSec = config.maxAngularVel().in(RadiansPerSecond);
        HUB_AIM_KP.onChange(headingPID::setP);
        HUB_AIM_KD.onChange(headingPID::setD);
        headingPID.enableContinuousInput(-Math.PI, Math.PI);
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

    public SwerveRequest getSwerveRequest(Rotation2d heading, Optional<Rotation2d> target) {
        if (target.isEmpty()) return getSwerveRequest();
        double scalar = swerveSpeedModifier();
        forward = forwardLimiter.calculate(-getLeftY() * scalar);
        strafe = strafeLimiter.calculate(-getLeftX() * scalar);
        double targetRad = angleModulus(target.get().getRadians());
        double radiansPerSec = headingFilter.calculate((targetRad - prevTargetRad) / 0.02)
            + headingPID.calculate(angleModulus(heading.getRadians()), targetRad);
        prevTargetRad = targetRad;
        return swerveReq
            .withVelocityX(forward * maxVelMetersPerSec / 2.0)
            .withVelocityY(strafe * maxVelMetersPerSec / 2.0)
            .withDeadband(0.05 * scalar * maxVelMetersPerSec)
            .withRotationalRate(radiansPerSec)
            .withRotationalDeadband(0);
    }

    // For Rumble to work on PS5 Controllers, we have to run a custom script on the driver station computer.
    // (scripts/driverstation/ps5_controller_rumble.py).
    @Override
    public void setRumble(GenericHID.RumbleType type, double value) {
        int scaled = (int) (MathUtil.clamp(value, 0, 1) * 255);
        switch (type) {
            case kLeftRumble -> leftRumble.set(scaled);
            case kRightRumble -> rightRumble.set(scaled);
            case kBothRumble -> {
                leftRumble.set(scaled);
                rightRumble.set(scaled);
            }
        }
    }
}