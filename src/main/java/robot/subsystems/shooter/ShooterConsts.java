package robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import lib.Tunable;
import lib.TunableLerpTable;
import robot.subsystems.common.PivotHardware;
import robot.subsystems.shooter.math.ShotMap;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static lib.Convert.CustomUnits.PoundSquareInches;

public class ShooterConsts {
    /** The yaw, pitch, and shooter speed for launched balls to reach the target. */
    public record ShooterSetpoint(
        Rotation2d yaw,
        Rotation2d pitch,
        AngularVelocity targetVelocity
    ) {}

    /** The optimal hood angle and travel time calculated from a {@link ShotMap}. */
    public record ShotMapResult(double pitchRad, double airTimeSecs) {}

    public static final ShooterSetpoint NULL_SHOOTER_SETPOINT =
        new ShooterSetpoint(Rotation2d.kZero, Rotation2d.kZero, RadiansPerSecond.zero());

    static final double HOOD_REDUCTION = 1.0; // TODO
    static final PivotHardware.PivotSimConfig HOOD_SIM_CFG = new PivotHardware.PivotSimConfig(
        HOOD_REDUCTION, PoundSquareInches.of(59.04), Inches.of(7.5),
        DCMotor.getNeo550(1), true
    );

    // TODO all of these have to be tuned:
    public static final TrapezoidProfile.Constraints HOOD_MOTION_CONSTRAINTS =
        new TrapezoidProfile.Constraints(100, 100);
    public static final double SPEED_TOLERANCE_MPS = 0.01, PHASE_DELAY = 0.03;
    public static final Transform2d ROBOT_TO_SHOOTER = new Transform2d(0.0, 0.0, Rotation2d.kZero);
    public static final Tunable<Double>
        DRAG_COMPENSATION = Tunable.of("Shooter/Drag Compensations(secs^-1)", 0.2),
        LOOKAHEAD_SECS = Tunable.of("Shooter/Lookahead time(secs)", 0),
        NEWTONS_METHOD_ITERATIONS = Tunable.of("Shooter/Newtons Method Iterations", 10);
    // Because of energy loss, the linear velocity of the ball isn't exactly
    // omega * r; so, we compensate for that with a lerp table.
    public static final TunableLerpTable ANGULAR_TO_LINEAR_VEL =
        new TunableLerpTable("Shooter/AngularVel(rad per sec) to LinearVel(mps)")
            .put(0, 0)
            .put(1, 1)
            .put(2, 2);
}
