package robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import robot.subsystems.shooter.math.ShotMap;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class DataTypes {
    /** The yaw, pitch, and shooter speed for launched balls to reach the target. */
    public record ShooterSetpoint(
        Rotation2d yaw,
        Rotation2d pitch,
        double speedRadPerSec,
        boolean flywheelsAreReady
    ) {
        /** Represents a ShooterSetpoint with no data. */
        public static final ShooterSetpoint NULL = new ShooterSetpoint(Rotation2d.kZero, Rotation2d.kZero, 0, false);

        public AngularVelocity speed() {
            return RadiansPerSecond.of(speedRadPerSec);
        }
    }

    /**
     * The optimal hood angle and travel time calculated from a {@link ShotMap}.
     * While ShooterSetpoint is directly used by the shooter, ShotMapResult
     * is used for internal calculations.
     */
    public record ShotMapResult(double pitchRad, double airTimeSecs) {}
}
