package lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;

/** A simulated predictive current limiter. */
public record SimCurrentLimit(DCMotor motorKind, Current limit) {
    /**
     * Returns a new voltage that respects the current limit.
     * @param rawVelocity The mechanism's velocity without any gear ratio.
     */
    public double calcVolts(double inputVolts, AngularVelocity rawVelocity) {
        double torque = motorKind.getTorque(limit.in(Amps));
        double minVolts = motorKind.getVoltage(-torque, rawVelocity.in(RadiansPerSecond));
        double maxVolts = motorKind.getVoltage(torque, rawVelocity.in(RadiansPerSecond));
        return MathUtil.clamp(inputVolts, minVolts, maxVolts);
    }
}
