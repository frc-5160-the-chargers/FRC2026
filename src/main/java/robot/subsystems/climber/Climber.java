package robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import robot.subsystems.ChargerSubsystem;

public class Climber extends ChargerSubsystem{
    private ClimberData data = new ClimberData();
    private SimClimberHardware io = new SimClimberHardware();

    @Override
    public void periodic() {
        io.refreshData(data);
        Logger.processInputs(getName(), data);
    }

    public Command climb() {
        return this.run(() -> io.setVoltage(6.0));
    }
}
