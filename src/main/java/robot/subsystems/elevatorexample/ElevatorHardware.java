package robot.subsystems.elevatorexample;

import lib.hardware.MotorStats;
import org.littletonrobotics.junction.AutoLog;

/**
 * A class that controls the hardware powering the elevator(motors and encoders).
 */
public class ElevatorHardware {
    @AutoLog
    static class ElevatorData {
        public double radians = 0, radiansPerSec = 0;
        public MotorStats leaderStats = MotorStats.EMPTY;
        public MotorStats followerStats = MotorStats.EMPTY;
    }

    public void refreshData(ElevatorDataAutoLogged data) {}

    public void setRadians(double radians, double feedforwardV) {}

    public void setVolts(double volts) {}

    public void setPDGains(double p, double d) {}

    public void zeroEncoder() {}
}