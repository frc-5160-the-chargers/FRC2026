package robot.subsystems.common;

import lib.hardware.MotorStats;
import org.littletonrobotics.junction.AutoLog;

/**
 * A hardware base class representing a generic roller mechanism.
 */
public class RollerHardware {
    @AutoLog
    static class RollerData {
        public double velocityRadPerSec = 0;
        public MotorStats[] motorStats = {MotorStats.EMPTY};
    }

    public void refreshData(RollerDataAutoLogged data) {}

    public void setVolts(double volts) {}

    public void setCurrentLimit(double amps) {}

    public void setCoastMode(boolean enabled) {}
}
