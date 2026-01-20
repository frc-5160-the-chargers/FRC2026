package robot.subsystems.common;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import lib.hardware.MotorStats;
import org.littletonrobotics.junction.AutoLog;

/**
 * A hardware base class representing a generic pivoting mechanism.
 */
public class PivotHardware {
    /** Data read from a pivoting mechanism every 0.02 secs. */
    @AutoLog
    static class PivotData {
        public double radians = 0, radiansPerSec = 0;
        public MotorStats motorStats = MotorStats.EMPTY;
    }

    /** Hardware-level config for a pivoting mechanism. */
    public record PivotSimConfig(
        double reduction,
        MomentOfInertia moi,
        double currentLimit,
        Distance pivotLength,
        DCMotor motorKind,
        boolean simulateGravity
    ) {}

    public void refreshData(PivotDataAutoLogged data) {}

    public void setRadians(double radians, double feedforwardV) {}

    public void setVolts(double volts) {}

    public void setPDGains(double p, double d) {}

    public void zeroEncoder() {}
}