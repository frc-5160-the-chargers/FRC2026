package lib.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import java.util.HashMap;

/**
 * Stores boilerplate statistics relating to motors. Since this class is a record class,
 * it can be logged within an @AutoLog annotated class.
 * Example Usage:
 * <pre><code>
 *     var spark = new SparkMax(...);
 *     var talonFx = new TalonFX(...);
 *     var sim = new DCMotorSim(...);
 *     inputs.motorStats = MotorStats.from(spark);
 *     inputs.motorStats = MotorStats.from(talonFx);
 *     inputs.motorStats = MotorStats.from(sim);
 *     inputs.motorStats = new MotorStats(...); // for custom motors/data
 *     System.out.println(inputs.motorStats.appliedVolts());
 * </code></pre>
 */
public record MotorStats(
    double appliedVolts, double tempCelsius,
    double supplyAmps, double appliedAmps,
    boolean connected
) {
    private static final HashMap<CommonTalon, BaseStatusSignal[]> talonSignalsCache = new HashMap<>();

    /** A default instance of {@link MotorStats} without any data. */
    public static final MotorStats EMPTY = new MotorStats(0, 0, 0, 0, true);

    public static MotorStats from(SparkBase motor) {
        return new MotorStats(
            motor.getAppliedOutput() * motor.getBusVoltage(),
            motor.getMotorTemperature(),
            0.0, // sparks don't log this value
            motor.getOutputCurrent(),
            !motor.hasActiveFault()
        );
    }

    public static MotorStats from(CommonTalon motor) {
        var signals = talonSignalsCache.get(motor);
        if (signals == null) {
            signals = new BaseStatusSignal[] {
                motor.getMotorVoltage(), motor.getDeviceTemp(),
                motor.getSupplyCurrent(), motor.getTorqueCurrent()
            };
            SignalRefresh.register(100, motor.getNetwork(), signals);
            BaseStatusSignal.setUpdateFrequencyForAll(50, signals[1], signals[2]);
            talonSignalsCache.put(motor, signals);
        }
        return new MotorStats(
            signals[0].getValueAsDouble(), signals[1].getValueAsDouble(),
            signals[2].getValueAsDouble(), signals[3].getValueAsDouble(),
            signals[1].getStatus().isOK()
        );
    }

    public static MotorStats from(DCMotorSim sim) {
        double amps = sim.getCurrentDrawAmps();
        return new MotorStats(sim.getInput(0), 0, amps, amps, true);
    }

    public static MotorStats from(SingleJointedArmSim sim) {
        double amps = sim.getCurrentDrawAmps();
        return new MotorStats(sim.getInput(0), 0, amps, amps, true);
    }

    public static MotorStats from(ElevatorSim sim) {
        double amps = sim.getCurrentDrawAmps();
        return new MotorStats(sim.getInput(0), 0, amps, amps, true);
    }
}