package robot.subsystems.intakepivot;

import lib.hardware.MotorStats;
import org.littletonrobotics.junction.AutoLog;

/**
 * A class that controls the hardware powering the elevator(motors and encoders).
 */
public class PivotHardware {
    @AutoLog
    static class PivotData {
        public double radians = 0, radiansPerSec = 0;
        public MotorStats motorStats = MotorStats.EMPTY;
    }

    public void refreshData(PivotDataAutoLogged data) {}

    public void setRadians(double radians, double feedforwardV) {}

    public void setVolts(double volts) {}

    public void setPDGains(double p, double d) {}

    public void zeroEncoder() {}
}