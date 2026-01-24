package robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public class ClimberHardware {
    @AutoLog
    public static class ClimberData {
        public double volts = 0.0;
        public double radians = 0.0;
    }

    public void refreshData(ClimberData data) {}

    public void setVoltage(double volts) {}

    public void setRadians(double radians) {}

    public void configureMotor() {}

    public void setPDGains(double p, double d) {}
}
