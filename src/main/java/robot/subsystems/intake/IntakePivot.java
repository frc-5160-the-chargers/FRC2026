package robot.subsystems.intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import lib.hardware.MotorStats;
import org.littletonrobotics.junction.AutoLog;
import robot.subsystems.common.PivotDataAutoLogged;

public class IntakePivot {
    @AutoLog
    static class PivotData {
        public double positionRad = 0, velocityRadPerSec = 0;
        public MotorStats motorStats = MotorStats.EMPTY;

        public TrapezoidProfile.State getMotionState() {
            return new TrapezoidProfile.State(positionRad, velocityRadPerSec);
        }
    }

    public void refreshData(PivotDataAutoLogged data) {}

    public void setRadians(double radians, double feedforwardV) {}

    public void setVolts(double volts) {}

    public void setPDGains(double p, double d) {}

    public void setCurrentLimit(double amps) {}

    public void zeroEncoder(double radians) {}
}