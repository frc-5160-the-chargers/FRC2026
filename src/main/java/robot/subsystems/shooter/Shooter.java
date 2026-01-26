package robot.subsystems.shooter;

import lib.RobotMode;
import robot.subsystems.ChargerSubsystem;
import robot.subsystems.common.PivotHardware;
import robot.subsystems.shooter.flywheels.FlywheelHardware;
import robot.subsystems.shooter.flywheels.KrakenFlywheelHardware;
import robot.subsystems.shooter.flywheels.SimFlywheelHardware;

public class Shooter extends ChargerSubsystem {
    private FlywheelHardware flywheelIO = switch (RobotMode.get()) {
        case REAL -> new KrakenFlywheelHardware();
        case SIM -> new SimFlywheelHardware();
        case REPLAY -> new FlywheelHardware();
    };
    private PivotHardware hoodIO = switch (RobotMode.get()) {
        case REAL -> new HoodHardware();
        case SIM, REPLAY -> new PivotHardware();
    };
}
