package robot.subsystems.serializer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.RobotMode;
import lib.Tunable;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.ChargerSubsystem;
import robot.subsystems.common.RollerDataAutoLogged;
import robot.subsystems.common.RollerHardware;
import robot.subsystems.common.SimRollerHardware;

import java.util.Optional;

public class Serializer extends ChargerSubsystem {
    static final double CURRENT_LIMIT = 60;
    static final double PULSE_ON_TIME_SECS = 2.0;
    static final double PULSE_OFF_TIME_SECS = 0.5;

    private final Tunable<Double>
        defaultVolts = Tunable.of(key("DefaultVolts"), 9.0),
        pulseOffVolts = Tunable.of(key("PulseOffVolts"), -3.0);
    private final RollerHardware io = switch (RobotMode.get()) {
        case REAL -> new KrakenSerializerRollers();
        case SIM -> new SimRollerHardware(DCMotor.getKrakenX60(1), 1.0);
        case REPLAY -> new RollerHardware();
    };
    private final RollerDataAutoLogged inputs = new RollerDataAutoLogged();
    private final Optional<IntakeSimulation> hopperSim;

    public Serializer(Optional<IntakeSimulation> hopperSim) {
        this.hopperSim = hopperSim;
    }

    public Command pulseCmd() {
        var cmd = Commands.repeatingSequence(
            this.run(() -> setVolts(defaultVolts.get()))
                .withTimeout(PULSE_ON_TIME_SECS),
            this.run(() -> setVolts(pulseOffVolts.get()))
                .withTimeout(PULSE_OFF_TIME_SECS)
        );
        return logged(cmd, "Pulse");
    }

    public Command runCmd() {
        var cmd = this.run(() -> setVolts(defaultVolts.get()));
        return logged(cmd, "Run");
    }

    public Command stopCmd() {
        var cmd = this.run(() -> setVolts(0));
        return logged(cmd, "Idle");
    }

    private void setVolts(double volts) {
        io.setVolts(volts);
        if (volts != 0.0 && hopperSim.isPresent()) {
            Logger.recordOutput(key("HasFuelInSim"), hopperSim.get().obtainGamePieceFromIntake());
        }
    }

    @Override
    public void periodic() {
        io.refreshData(inputs);
        Logger.processInputs(getName(), inputs);
    }
}
