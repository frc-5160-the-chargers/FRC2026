package robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Tracer;
import org.littletonrobotics.junction.Logger;

public abstract class ChargerSubsystem extends SubsystemBase {
    /** A constructor for ChargerSubsystem that uses a custom subsystem name. */
    public ChargerSubsystem(String name) { super(name); }

    /** The default constructor for ChargerSubsystem. */
    public ChargerSubsystem() {}

    /** A convenience method for fetching a relative logging key for this subsystem. */
    public String key(String path) {
        return super.getName() + "/" + path;
    }

    /**
     * If you override this method instead of periodic(),
     * extra logging will be added by default.
     */
    public void loggedPeriodic() {}

    @Override
    public void periodic() {
        Tracer.startTrace(getName() + " Periodic");
        loggedPeriodic();
        var currCmd = super.getCurrentCommand();
        Logger.recordOutput(
            key("CurrentCommand"),
            currCmd == null ? "none" : currCmd.getName()
        );
        Tracer.endTrace();
    }
}
