package robot.subsystems.serializer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import lib.RobotMode;
import lib.Tunable;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.ChargerSubsystem;
import robot.subsystems.common.RollerDataAutoLogged;
import robot.subsystems.common.RollerHardware;
import robot.subsystems.common.SimRollerHardware;

public class Serializer extends ChargerSubsystem {
    static final double CURRENT_LIMIT = 60;

    private final Tunable<Double>
        forwardVolts = Tunable.of(key("ForwardVolts"), 9.0),
        reverseVolts = Tunable.of(key("ReverseVolts"), -6.0);
    private final RollerHardware io = switch (RobotMode.get()) {
        case REAL -> new KrakenSerializerRollers();
        case SIM -> new SimRollerHardware(DCMotor.getKrakenX60(1), 1.0);
        case REPLAY -> new RollerHardware();
    };
    private final RollerDataAutoLogged inputs = new RollerDataAutoLogged();

    public Command runCmd() {
        var cmd = this.run(() -> io.setVolts(forwardVolts.get()));
        return logged(cmd, "RunForward");
    }

    public Command runReverseCmd() {
        var cmd = this.run(() -> io.setVolts(reverseVolts.get()));
        return logged(cmd, "RunForward");
    }

    @Override
    public void periodic() {
        io.refreshData(inputs);
        Logger.processInputs(getName(), inputs);
    }
}
