package robot.subsystems.shooter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lib.Convert;
import lib.RobotMode;
import lib.Tunable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.ChargerSubsystem;
import robot.subsystems.common.PivotController;
import robot.subsystems.common.PivotController.PivotConstraints;
import robot.subsystems.common.PivotController.PivotGains;
import robot.subsystems.common.PivotDataAutoLogged;
import robot.subsystems.common.PivotHardware;
import robot.subsystems.common.PivotHardware.PivotSimConfig;
import robot.subsystems.common.SimPivotHardware;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static lib.Convert.CustomUnits.PoundSquareInches;

public class Shooter extends ChargerSubsystem {
    /** The yaw, pitch, and shooter speed for launched balls to reach the target. */
    public record Setpoint(Rotation2d yaw, Rotation2d pitch, double radPerSec) {
        /** Represents a Shooter setpoint with no data. */
        public static final Setpoint NULL = new Setpoint(Rotation2d.kZero, Rotation2d.kZero, 0);

        public AngularVelocity speed() {
            return RadiansPerSecond.of(radPerSec);
        }
    }

    static final double HOOD_REDUCTION = 25.0;
    static final DCMotor HOOD_MOTOR_KIND = DCMotor.getNeo550(1);
    static final PivotSimConfig HOOD_SIM_CFG = new PivotSimConfig(
        HOOD_REDUCTION, PoundSquareInches.of(59.04),
        Inches.of(7.5), HOOD_MOTOR_KIND, false
    );
    static final double HOOD_KV = 1 / HOOD_MOTOR_KIND.withReduction(HOOD_REDUCTION).KvRadPerSecPerVolt;
    static final PivotConstraints HOOD_CONSTRAINTS = new PivotConstraints(5, 3);
    static final PivotGains HOOD_GAINS = new PivotGains(0, HOOD_KV, 0, 4.0, 0);

    static final double FLYWHEEL_REDUCTION = 1.0;

    public static final Transform2d ROBOT_TO_LAUNCH_POINT =
        new Transform2d(Inches.of(6.07), Inches.zero(), Rotation2d.kZero);
    public static final Translation3d ROBOT_TO_SHOOTER_PIVOT_POINT =
        new Translation3d(Inches.of(9.06), Inches.zero(), Inches.of(11.776));
    public static final Distance FUEL_LAUNCH_HEIGHT = Inches.of(15.2);

    private final Tunable<Double>
        flywheelKp = Tunable.of(key("Flywheels/KP"), 7.0),
        flywheelKd = Tunable.of(key("Flywheels/KD"), 0.0),
        flywheelKv = Tunable.of(key("Flywheels/KV"), 0.0),
        hoodTolerance = Tunable.of(key("Hood/Tolerance (Rad)"), 1 * Convert.DEGREES_TO_RADIANS);
    private final KrakenFlywheels flywheelIO = new KrakenFlywheels();
    private final PivotHardware hoodIO = switch (RobotMode.get()) {
        case REAL -> new NeoShooterHood();
        case SIM -> new SimPivotHardware(HOOD_SIM_CFG);
        case REPLAY -> new PivotHardware();
    };
    private final FlywheelDataAutoLogged flywheelInputs = new FlywheelDataAutoLogged();
    private final PivotDataAutoLogged hoodInputs = new PivotDataAutoLogged();

    private final PivotController hoodController = new PivotController(
        key("Hood"), HOOD_GAINS, HOOD_CONSTRAINTS, hoodIO
    );

    /** Whether the shooter hood is at it's desired goal. */
    @AutoLogOutput public boolean hoodAtGoal = false;

    public final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(1),
            Seconds.of(20),
            state -> Logger.recordOutput(key("SysIdState"), state)
        ),
        new SysIdRoutine.Mechanism(
            output -> flywheelIO.setAmps(output.in(Volts)),
            null,
            this
        )
    );

    public Shooter() {
        configureGains();
        flywheelKp.onChange(this::configureGains);
        flywheelKd.onChange(this::configureGains);
        flywheelKv.onChange(this::configureGains);
        hoodIO.zeroEncoder(0 * Convert.DEGREES_TO_RADIANS);
    }

    private void configureGains() {
        flywheelIO.setGains(flywheelKp.get(), flywheelKd.get(), flywheelKv.get());
    }

    public Command stopCmd() {
        var cmd = this.run(() -> {
            flywheelIO.idle();
            hoodIO.setVolts(0);
        });
        return logged(cmd, "Idle");
    }

    /** Runs the shooter at the commanded setpoint. */
    public Command runCmd(Supplier<Setpoint> targetSupplier) {
        var resetStateCmd = this.runOnce(() -> hoodController.resetTo(hoodInputs.getMotionState()));
        var targetGoalCmd = this.run(() -> {
            var target = targetSupplier.get();
            var hoodGoal = new TrapezoidProfile.State(Math.PI / 2 - target.pitch().getRadians(), 0);
            flywheelIO.setVelocity(target.speed());
            hoodController.moveTo(hoodGoal, hoodInputs.positionRad);
            hoodAtGoal = Math.abs(hoodInputs.positionRad - hoodGoal.position) < hoodTolerance.get();
        });
        return logged(resetStateCmd.andThen(targetGoalCmd), "Run");
    }

    public Command manualHoodCmd(DoubleSupplier volts) {
        var cmd = this.run(() -> hoodIO.setVolts(volts.getAsDouble()));
        return logged(cmd, "ManualHood");
    }

    public Command setHoodAngleCmd(Supplier<Rotation2d> angle) {
        return runCmd(() -> new Setpoint(Rotation2d.kZero, angle.get(), 0));
    }

    @Override
    public void loggedPeriodic() {
        flywheelIO.refreshData(flywheelInputs);
        hoodIO.refreshData(hoodInputs);
        Logger.processInputs("Shooter/Flywheels", flywheelInputs);
        Logger.processInputs("Shooter/Hood", hoodInputs);

        var hoodPos = new Pose3d(
            ROBOT_TO_SHOOTER_PIVOT_POINT,
            new Rotation3d(0, hoodInputs.positionRad - 11 * Convert.DEGREES_TO_RADIANS, 0)
        );
        Logger.recordOutput("HoodPosition", hoodPos);
    }
}
