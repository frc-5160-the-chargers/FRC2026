package lib;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.HashMap;
import java.util.function.Supplier;

/**
 * A replacement for {@link choreo.auto.AutoChooser} that works with AdvantageKit.
 * In general, loading autonomous routines takes time due to parsing JSON.
 * This class allows you to only load in auto trajectories after they're selected,
 * preventing lag spikes in the robot code.
 */
public class LoggedAutoChooser {
    private final LoggedDashboardChooser<String> impl;
    private final HashMap<String, Supplier<Command>> choices = new HashMap<>();
    private Command selectedAuto = Commands.none();

    public LoggedAutoChooser(String name) {
        impl = new LoggedDashboardChooser<>(name);
        impl.onChange(option -> selectedAuto = choices.get(option).get());
        choices.put("No Auto", Commands::none);
        impl.addDefaultOption("No Auto", "No Auto");
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