package robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import lib.Tunable;

import static edu.wpi.first.units.Units.*;

public class ClimberConsts {
    static final double REDUCTION = 5.0;
    static final Mass CLIMBER_MASS = Pounds.of(7);
    static final DCMotor MOTOR_KIND = DCMotor.getNeoVortex(2);

    static final Distance TOLERANCE = Inches.of(0.5);
    static final Distance COG_LOW_BOUNDARY = Meters.of(0.6);
    static final Distance MAX_HEIGHT = Meters.of(1.285);
    static final Distance MIN_HEIGHT = Meters.of(-0.01);

    static final Tunable<Double> KP = Tunable.of("Elevator/KP", 0.5);
    static final Tunable<Double> KD = Tunable.of("Elevator/KD", 0);
}
