package robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import lib.hardware.MotorStats;

public class ClimberHardware {
    @AutoLog
    public static class ClimberData {
        public MotorStats motorStats = MotorStats.EMPTY;
        public double radians = 0.0;
    }

    public void refreshData(ClimberData data) {}

    public void setVoltage(double volts) {}

    public void setRadians(double radians) {}

    public void configureMotor() {}

    public void setPDGains(double p, double d) {}
}
