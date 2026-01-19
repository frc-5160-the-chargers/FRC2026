package robot.subsystems.common;

import lib.hardware.MotorStats;
import org.littletonrobotics.junction.AutoLog;
import robot.subsystems.rollers.RollerDataAutoLogged;

public class RollerHardware {
    @AutoLog
    static class RollerData {
        public double radiansPerSec = 0;
        public MotorStats[] motorStats = {MotorStats.EMPTY};
    }

    public void refreshData(RollerDataAutoLogged data) {}

    public void setVolts(double volts) {}

    public void setCurrentLimit(double amps) {}

    public void setCoastMode(boolean enabled) {}
}
