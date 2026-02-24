package robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lib.RobotMode;
import lib.Tunable;
import lib.TunableLerpTable;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.ChargerSubsystem;
import robot.subsystems.common.PivotController;
import robot.subsystems.common.PivotController.PivotConstraints;
import robot.subsystems.common.PivotController.PivotGains;
import robot.subsystems.common.PivotDataAutoLogged;
import robot.subsystems.common.PivotHardware;
import robot.subsystems.common.PivotHardware.PivotSimConfig;
import robot.subsystems.common.SimPivotHardware;
import robot.subsystems.shooter.DataTypes.ShooterSetpoint;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static lib.Convert.CustomUnits.PoundSquareInches;

public class Shooter extends ChargerSubsystem {
    static final double HOOD_REDUCTION = 1.0;
    static final DCMotor HOOD_MOTOR_KIND = DCMotor.getNeo550(1);
    static final PivotSimConfig HOOD_SIM_CFG = new PivotSimConfig(
        HOOD_REDUCTION, PoundSquareInches.of(59.04),
        Inches.of(7.5), HOOD_MOTOR_KIND, true
    );
    static final double HOOD_KV = 1 / HOOD_MOTOR_KIND.withReduction(HOOD_REDUCTION).KvRadPerSecPerVolt;
    static final PivotConstraints HOOD_CONSTRAINTS = new PivotConstraints(100, 100);
    static final PivotGains HOOD_GAINS = new PivotGains(0, HOOD_KV, 0, 5.0, 0);

    public static final Transform2d ROBOT_TO_SHOOTER = new Transform2d(0.0, 0.0, Rotation2d.kZero);
    public static final Tunable<Double>
        DRAG_COMPENSATION = Tunable.of("Shooter/Drag Compensations(secs^-1)", 0.2),
        LOOKAHEAD_SECS = Tunable.of("Shooter/Lookahead time(secs)", 0),
        NEWTONS_METHOD_ITERATIONS = Tunable.of("Shooter/Newtons Method Iterations", 10),
        FLYWHEEL_KP = Tunable.of("Shooter/Flywheels/KP", 5.0),
        FLYWHEEL_KD = Tunable.of("Shooter/Flywheels/KD", 0.0),
        FLYWHEEL_KV = Tunable.of("Shooter/Flywheels/KV", 0.0);
    // Because of energy loss, the linear velocity of the ball isn't exactly
    // omega * r; so, we compensate for that with a lerp table.
    public static final TunableLerpTable ANGULAR_TO_LINEAR_VEL =
        new TunableLerpTable("Shooter/AngularVel(rad per sec) to LinearVel(mps)")
            .put(0, 0)
            .put(1, 1)
            .put(2, 2);

    private final FlywheelHardware flywheelIO = switch (RobotMode.get()) {
        case REAL -> new KrakenFlywheels();
        case SIM, REPLAY -> new FlywheelHardware();
    };
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

    @Getter
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
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
        FLYWHEEL_KP.onChange(this::configureGains);
        FLYWHEEL_KD.onChange(this::configureGains);
        FLYWHEEL_KV.onChange(this::configureGains);
    }

    private void configureGains() {
        flywheelIO.setGains(FLYWHEEL_KP.get(), FLYWHEEL_KD.get(), FLYWHEEL_KV.get());
    }

    /** The current flywheel velocity. */
    public AngularVelocity getVelocity() {
        return flywheelInputs.velocity;
    }

    /** Runs the shooter at the commanded setpoint. */
    public Command runCmd(Supplier<ShooterSetpoint> targetSupplier) {
        var resetStateCmd = this.runOnce(() -> hoodController.resetTo(hoodInputs.getMotionState()));
        var targetGoalCmd = this.run(() -> {
            var target = targetSupplier.get();
            var hoodGoal = new TrapezoidProfile.State(target.pitch().getRadians(), 0);
            flywheelIO.setVelocity(target.velocity());
            hoodController.moveTo(hoodGoal, hoodInputs.positionRad);
        });
        return logged(resetStateCmd.andThen(targetGoalCmd), "Run");
    }

    @Override
    public void loggedPeriodic() {
        flywheelIO.refreshData(flywheelInputs);
        hoodIO.refreshData(hoodInputs);
        Logger.processInputs("Shooter/Flywheels", flywheelInputs);
        Logger.processInputs("Shooter/Hood", hoodInputs);
    }
}
