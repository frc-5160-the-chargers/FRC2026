package robot;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.commands.CmdLogger;
import robot.constants.LoggingConfig;
import robot.misc.DriverController;
import robot.subsystems.drive.SwerveConfig;
import robot.subsystems.drive.TunerConstants;

public class ControllerTesting extends LoggedRobot {
    static {
        LoggingConfig.initForMainRobot();
    }

    private final SwerveConfig swerveCfg = new SwerveConfig(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft, TunerConstants.FrontRight,
        TunerConstants.BackLeft, TunerConstants.BackRight
    );

    DriverController controller = new DriverController(0, swerveCfg);
    
    public Command rumble() {
        return Commands.run(() -> controller.setRumble(RumbleType.kLeftRumble,0.5))
        .withTimeout(10)
        .finallyDo(() -> controller.setRumble(RumbleType.kLeftRumble, 0));
    }

    public ControllerTesting() {
        controller.cross().whileTrue(rumble());
    }

    @Override
    public void robotPeriodic() {
        CmdLogger.periodic(true);
        CommandScheduler.getInstance().run();
    }
}
