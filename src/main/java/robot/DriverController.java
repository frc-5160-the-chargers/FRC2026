package robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import lib.Tunable;
import robot.subsystems.drive.SwerveConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.Alert.AlertType.kWarning;
import static lib.commands.TriggerUtil.bind;

public class DriverController extends CommandPS5Controller {
    private final double maxVelMetersPerSec, maxVelRadPerSec;
    private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity);
    private final Tunable<Double> speedReduction = Tunable.of("SpeedReduction", 1);

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
}