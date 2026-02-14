package robot.subsystems.serializer;

import edu.wpi.first.wpilibj2.command.Command;
import robot.subsystems.ChargerSubsystem;

public class Serializer extends ChargerSubsystem {
    public Command runCmd() {
        return this.run(() -> {});
    }
}
