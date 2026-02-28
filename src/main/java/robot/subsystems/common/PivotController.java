package robot.subsystems.common;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import lib.Tunable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** A higher-level base class for controlling a pivoting mechanism. */
public class PivotController {
    /** Velocity & Accel constraints for a pivot. */
    public record PivotConstraints(double maxVelRadPerSec, double maxAccelRadPerSecSq) {}

    /** Feedforward and PID gains for a pivot. */
    public record PivotGains(double kS, double kV, double kG, double kP, double kD) {}

    private final String name;
    private final PivotHardware io;
    private final Tunable<Double> maxAccel, maxVel, kS, kG, kP;
    private final double kV, kD;
    private TrapezoidProfile motionProfile;

    @AutoLogOutput(key = "{name}/setpoint")
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public PivotController(String name, PivotGains gains, PivotConstraints constraints, PivotHardware io) {
        this.name = name;
        this.io = io;
        maxVel = Tunable.of(name + "/MaxVel(rad per s)", constraints.maxVelRadPerSec);
        maxAccel = Tunable.of(name + "/MaxAccel(rad per s^2)", constraints.maxAccelRadPerSecSq);
        kS = Tunable.of(name + "/Gains/KS(Volts)", gains.kS);
        kG = Tunable.of(name + "/Gains/KG(Volts)", gains.kG);
        kP = Tunable.of(name + "/Gains/KP", gains.kP);
        kV = gains.kV;
        kD = gains.kD;

        applyConfigs();
        maxVel.onChange(this::applyConfigs);
        maxAccel.onChange(this::applyConfigs);
        kP.onChange(this::applyConfigs);
    }

    private void applyConfigs() {
        var constraints = new TrapezoidProfile.Constraints(maxVel.get(), maxAccel.get());
        motionProfile = new TrapezoidProfile(constraints);
        io.setPDGains(kP.get(), kD);
    }

    /** Resets the pivoting mechanism's state. Should be called before moveTo() is called within a command. */
    public void resetTo(TrapezoidProfile.State state) {
        setpoint = state;
    }

    /** Directs the pivoting mechanism to a specific goal. */
    public void moveTo(TrapezoidProfile.State goal, double pivotPositionRad) {
        setpoint = motionProfile.calculate(0.02, setpoint, goal);
        double ff = kG.get() * Math.cos(pivotPositionRad); // pivotInputs.radians must be 0 degrees when horizontal.
        ff += Math.signum(setpoint.velocity) * kS.get();
        ff += kV * setpoint.velocity;
        Logger.recordOutput(name + "/PivotFF", ff);
        io.setRadians(setpoint.position, ff);
    }

    public double getAntiGravityVolts(double pivotPositionRad) {
        return kG.get() * Math.cos(pivotPositionRad);
    }
}
