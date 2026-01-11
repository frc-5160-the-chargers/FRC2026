package lib.commands;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * A replacement for {@link choreo.auto.AutoChooser} that works with AdvantageKit.
 * In general, loading autonomous routines takes time due to parsing JSON.
 * This class allows you to only load in auto trajectories after they're selected,
 * preventing lag spikes in the robot code. <br />
 *
 * Note that for this chooser to work in simulation, you must change the driver station mode
 * from "disconnected" to "disabled", then from "disabled" to "autonomous".
 */
public class LoggedAutoChooser {
    private static final String NONE_NAME = "No Auto";

    private final LoggedDashboardChooser<String> impl;
    private final HashMap<String, Supplier<Command>> choices = new HashMap<>();
    private Optional<DriverStation.Alliance> alliance = Optional.empty();
    private Command selectedAuto = Commands.none();

    public LoggedAutoChooser(String name) {
        impl = new LoggedDashboardChooser<>(name);
        impl.onChange(this::onChange);
        new Trigger(() -> !DriverStation.getAlliance().equals(alliance))
            .and(DriverStation::isDisabled)
            .onTrue(
                Commands.runOnce(() -> onChange(impl.get()))
                    .ignoringDisable(true)
                    .withName("Auto Chooser Alliance Handler")
            );
        choices.put(NONE_NAME, Commands::none);
        impl.addDefaultOption(NONE_NAME, NONE_NAME);
    }

    private void onChange(String option) {
        alliance = DriverStation.getAlliance();
        selectedAuto = alliance.isEmpty() ? Commands.none() : choices.get(option).get();
    }

    /**
     * Adds a Command to the auto chooser.
     * @see choreo.auto.AutoChooser#addCmd
     */
    public void addCmd(String name, Supplier<Command> commandSupplier) {
        choices.put(name, commandSupplier);
        impl.addOption(name, name);
    }

    /**
     * Add an AutoRoutine to the chooser.
     * @see choreo.auto.AutoChooser#addRoutine
     */
    public void addRoutine(String name, Supplier<AutoRoutine> routineSupplier) {
        addCmd(name, () -> routineSupplier.get().cmd());
    }

    /**
     * A command that schedules the selected autonomous command.
     * Should be mapped to a trigger, like so:
     * <code>
     *     RobotModeTriggers.autonomous().whileTrue(chooser.autoScheduler());
     * </code>
     */
    public Command autoScheduler() {
        return Commands.deferredProxy(() -> selectedAuto).withName("Auto Command Scheduler");
    }
}