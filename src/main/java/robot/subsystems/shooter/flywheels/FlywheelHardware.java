package robot.subsystems.shooter.flywheels;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class FlywheelHardware {
    @AutoLog
    static class FlywheelData {
        public AngularVelocity velocity = RadiansPerSecond.zero();
    }

    public void setVoltage(double volts) {}

    public void setVelocity(AngularVelocity velocity) {}

    public void refreshData(FlywheelDataAutoLogged data) {}
}
