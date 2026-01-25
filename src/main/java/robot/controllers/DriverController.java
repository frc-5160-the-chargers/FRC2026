package robot.controllers;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import lib.Tunable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.drive.SwerveConfig;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.wpilibj.Alert.AlertType.kWarning;
import static lib.commands.TriggerUtil.bind;

public class DriverController extends CommandPS5Controller {
    private final Tunable<Double> speedReduction = Tunable.of("SpeedReduction", 1);
    private final double maxVelMetersPerSec, maxVelRadPerSec;
    private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity);

    public DriverController(SwerveConfig config) {
        this(0, config);
    }

    public DriverController(int port, SwerveConfig config) {
        super(port);
        this.maxVelMetersPerSec = config.maxVel().in(MetersPerSecond);
        this.maxVelRadPerSec = config.maxAngularVel().in(RadiansPerSecond);
        bind(new Alert("Driver Disconnected", kWarning), () -> !super.isConnected());
    }

    @AutoLogOutput
    private double slowModeOutput() {
        double output = getL2Axis();
        output = MathUtil.applyDeadband(output, 0.2, 1);
        output = (2 - output) / 2;
        return output;
    }

    private double modifyDriveAxis(double output) {
        output *= slowModeOutput() * speedReduction.get();
        return output * -1;
    }

    public SwerveRequest.FieldCentric getSwerveRequest() {
        double x = modifyDriveAxis(getLeftY());
        double y = modifyDriveAxis(getLeftX());
        double rot = modifyDriveAxis(getRightX());
        Logger.recordOutput("DriverController/xOutput", x);
        Logger.recordOutput("DriverController/yOutput", y);
        Logger.recordOutput("DriverController/rotOutput", rot);
        return request
            .withVelocityX(x * maxVelMetersPerSec)
            .withVelocityY(y * maxVelMetersPerSec)
            .withRotationalRate(rot * maxVelRadPerSec)
            .withDeadband(0.1 * maxVelMetersPerSec * speedReduction.get())
            .withRotationalDeadband(0.1 * maxVelRadPerSec * speedReduction.get());
    }

    // We use EpilogueBackend to log to publish to NT regardless of whether the FMS is enabled.
    private final EpilogueBackend rumblePublisher =
        new NTEpilogueBackend(NetworkTableInstance.getDefault()).getNested("ControllerRumble");

    // For Rumble to work on PS5 Controllers, we have to run a custom script on the driver station computer.
    // (scripts/driverstation/ps5_controller_rumble.py).
    @Override
    public void setRumble(GenericHID.RumbleType type, double value) {
        int scaled = (int) (MathUtil.clamp(value, 0, 1) * 255);
        switch (type) {
            case kLeftRumble -> rumblePublisher.log("LeftRequest", scaled);
            case kRightRumble -> rumblePublisher.log("RightRequest", scaled);
            case kBothRumble -> {
                rumblePublisher.log("LeftRequest", scaled);
                rumblePublisher.log("RightRequest", scaled);
            }
        }
    }
}