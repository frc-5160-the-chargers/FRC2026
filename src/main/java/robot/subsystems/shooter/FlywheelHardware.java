package robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import lib.hardware.MotorStats;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class FlywheelHardware {
    @AutoLog
    static class FlywheelData {
        public AngularVelocity velocity = RadiansPerSecond.zero();
        public double pidError = 0.0;
        public MotorStats leaderStats = MotorStats.EMPTY;
    }

    public void setAmps(double amps) {}

    public void setVelocity(AngularVelocity velocity) {}

    public void setGains(double kP, double kD, double kV) {}

    public void refreshData(FlywheelDataAutoLogged data) {}
}
