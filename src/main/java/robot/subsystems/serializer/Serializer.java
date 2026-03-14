package robot.subsystems.serializer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.RobotMode;
import lib.Tunable;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.ChargerSubsystem;
import robot.subsystems.common.RollerDataAutoLogged;
import robot.subsystems.common.RollerHardware;
import robot.subsystems.common.SimRollerHardware;

public class Serializer extends ChargerSubsystem {
    static final double CURRENT_LIMIT = 60;
    static final int SIM_FUEL_REMOVAL_RATE = 5; // # of game pieces per sec
    static final double PULSE_ON_TIME_SECS = 1.0;
    static final double PULSE_OFF_TIME_SECS = 0.5;
    public static final double FLYWHEEL_TO_SERIALIZER_SPEED_RATIO = 0.4;

    private final Tunable<Double>
        defaultVolts = Tunable.of(key("DefaultVolts"), 9.0),
        pulseOffVolts = Tunable.of(key("PulseOffVolts"), 3.0);
    private final RollerHardware io = switch (RobotMode.get()) {
        case REAL -> new KrakenSerializerRollers();
        case SIM -> new SimRollerHardware(DCMotor.getKrakenX60(1), 1.0);
        case REPLAY -> new RollerHardware();
    };
    private final RollerDataAutoLogged inputs = new RollerDataAutoLogged();

    public Command pulseCmd() {
        var cmd = Commands.repeatingSequence(
            this.run(() -> io.setVolts(defaultVolts.get()))
                .withTimeout(PULSE_ON_TIME_SECS),
            this.run(() -> io.setVolts(pulseOffVolts.get()))
                .withTimeout(PULSE_OFF_TIME_SECS)
        );
        return logged(cmd, "Pulse");
    }

    public Command runCmd() {
        var cmd = this.run(() -> io.setVolts(defaultVolts.get()));
        return logged(cmd, "Run");
    }

    public Command stopCmd() {
        var cmd = this.run(() -> io.setVolts(0));
        return logged(cmd, "Idle");
    }

    @Override
    public void periodic() {
        io.refreshData(inputs);
        Logger.processInputs(getName(), inputs);
    }
}
