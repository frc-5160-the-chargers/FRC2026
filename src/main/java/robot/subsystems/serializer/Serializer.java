package robot.subsystems.serializer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.RobotMode;
import lib.Tunable;
import lib.hardware.LaserCanDataAutoLogged;
import lib.hardware.LoggedLaserCan;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.ChargerSubsystem;
import robot.subsystems.common.RollerDataAutoLogged;
import robot.subsystems.common.RollerHardware;
import robot.subsystems.common.SimRollerHardware;

import java.util.Optional;
import java.util.function.BooleanSupplier;


public class Serializer extends ChargerSubsystem {
    static final double CURRENT_LIMIT = 60;
    static final int SIM_FUEL_REMOVAL_RATE = 5; // # of game pieces per sec
    static final double PULSE_ON_TIME_SECS = 1.0;
    static final double PULSE_OFF_TIME_SECS = 0.5;
    public static final double FLYWHEEL_TO_SERIALIZER_SPEED_RATIO = 0.4;

    private final Tunable<Double>
        forwardVolts = Tunable.of(key("ForwardVolts"), 9.0),
        reverseVolts = Tunable.of(key("ReverseVolts"), -6.0),
        laserCanThreshold = Tunable.of(key("LaserCanDistanceThreshold (mm)"), 10);
    private final RollerHardware io = switch (RobotMode.get()) {
        case REAL -> new KrakenSerializerRollers();
        case SIM -> new SimRollerHardware(DCMotor.getKrakenX60(1), 1.0);
        case REPLAY -> new RollerHardware();
    };
    private final RollerDataAutoLogged inputs = new RollerDataAutoLogged();

    private final LoggedLaserCan[] laserCans = {};
    private final LaserCanDataAutoLogged[] laserCanInputs = {};

    private final Optional<IntakeSimulation> sim;

    public Serializer(Optional<IntakeSimulation> sim) {
        this.sim = sim;
    }

    public Command pulseWhileTrueCmd(BooleanSupplier shouldRun) {
        var cmd = Commands.repeatingSequence(
            runWhileTrueCmd(shouldRun).withTimeout(PULSE_ON_TIME_SECS),
            stopCmd().withTimeout(PULSE_OFF_TIME_SECS)
        );
        return logged(cmd, "RunForward (Pulsing)");
    }

    public Command runWhileTrueCmd(BooleanSupplier shouldRun) {
        var debouncer = new Debouncer(0.1);
        var cmd = this.run(() -> {
            boolean shouldStop = debouncer.calculate(!shouldRun.getAsBoolean());
            Logger.recordOutput(key("IsStopped"), shouldStop);
            io.setVolts(shouldStop ? 0 : forwardVolts.get());
            removeFuelIfApplicable();
        });
        return logged(cmd, "RunForward");
    }

    public Command runReverseCmd() {
        var cmd = this.run(() -> {
            io.setVolts(reverseVolts.get());
            removeFuelIfApplicable();
        });
        return logged(cmd, "RunForward");
    }

    public Command stopCmd() {
        var cmd = this.run(() -> {
            io.setVolts(0);
            Logger.recordOutput(key("IsStopped"), true);
        });
        return logged(cmd, "Idle");
    }

    private void removeFuelIfApplicable() {
        if (sim.isEmpty()) return;
        Logger.runEveryN(
            (int) (1.0 / (SIM_FUEL_REMOVAL_RATE * 0.02)),
            () -> sim.get().obtainGamePieceFromIntake()
        );
    }

    @AutoLogOutput
    public boolean isEmpty() {
        if (sim.isPresent()) return sim.get().getGamePiecesAmount() == 0;
        for (int i = 0; i < laserCans.length; i++) {
            if (laserCanInputs[i].distance_mm < laserCanThreshold.get()) return false;
        }
        return true;
    }

    @Override
    public void periodic() {
        io.refreshData(inputs);
        Logger.processInputs(getName(), inputs);
        for (int i = 0; i < laserCans.length; i++) {
            laserCans[i].refreshData(laserCanInputs[i]);
            Logger.processInputs(key("LaserCans/" + i), laserCanInputs[i]);
        }
    }
}
