package robot.subsystems.intakepivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import lombok.experimental.FieldDefaults;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

@FieldDefaults(makeFinal = true)
public class PivotConsts {
    static double REDUCTION = 2.0;
    static double MOI_KG_METERS_SQ = 0.001;
    static double CURRENT_LIMIT_AMPS = 60;
    static Distance PIVOT_LENGTH = Meters.of(0.3);
    static DCMotor MOTOR_KIND = DCMotor.getNEO(1);
    static LinearVelocity MAX_VEL = MetersPerSecond.of(2);
}
