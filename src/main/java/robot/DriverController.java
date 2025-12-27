package robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import robot.subsystems.drive.SwerveConfig;
import robot.subsystems.drive.TunerConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.wpilibj.Alert.AlertType.kWarning;
import static lib.commands.TriggerUtil.bind;

public class DriverController extends CommandPS5Controller {
    private final double maxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxSpeedRadPerSec = maxSpeedMps / SwerveConfig.DEFAULT.drivebaseRadius().in(Meters);
    private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withDeadband(0.1 * maxSpeedMps)
        .withRotationalDeadband(0.1 * maxSpeedRadPerSec);

    public DriverController() {
        super(0);
    }

    public DriverController(int port) {
        super(port);
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
        output *= slowModeOutput();
        return output * -1;
    }

    public SwerveRequest.FieldCentric getSwerveRequest() {
        double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        double x = modifyDriveAxis(getLeftY());
        double y = modifyDriveAxis(getLeftX());
        double rot = modifyDriveAxis(getRightX());
        Logger.recordOutput("DriverController/xOutput", x);
        Logger.recordOutput("DriverController/yOutput", y);
        Logger.recordOutput("DriverController/rotOutput", rot);
        return request
            .withVelocityX(x * maxSpeed)
            .withVelocityY(y * maxSpeed)
            .withRotationalRate(rot * maxSpeed);
    }
}