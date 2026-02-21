package robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.RobotMode;
import lib.Tunable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.ChargerSubsystem;
import robot.subsystems.common.*;
import robot.subsystems.common.PivotHardware.PivotSimConfig;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static lib.Convert.CustomUnits.PoundSquareInches;

public class GroundIntake extends ChargerSubsystem {
    public static final Distance PIVOT_LENGTH = Inches.of(12.4);
    static final MomentOfInertia PIVOT_MOI = PoundSquareInches.of(1344.71);
    static final double PIVOT_REDUCTION = 25.0 * 24.0 / 22.0; // MaxPlanetary + Sprockets
    static final double ROLLER_REDUCTION = 1.0;

    static final DCMotor ROLLER_MOTOR_KIND = DCMotor.getNeoVortex(1);
    static final DCMotor PIVOT_MOTOR_KIND = DCMotor.getNEO(1);
    static final double PIVOT_KV = 1 / PIVOT_MOTOR_KIND.withReduction(PIVOT_REDUCTION).KvRadPerSecPerVolt;
    static final double PIVOT_KD = 0.01;

    private final Tunable<Double>
        rollerVolts = Tunable.of(key("Rollers/Power(Volts)"), 3.5),
        rollerCurrentLimit = Tunable.of(key("Rollers/CurrentLimit(Amps)"), 55);
    private final Tunable<Double> // TODO tune
        pivotMaxVel = Tunable.of(key("Pivot/MaxVel(rad per s)"), 5),
        pivotMaxAccel = Tunable.of(key("Pivot/MaxAccel(rad per s^2)"), 13),
        pivotKs = Tunable.of(key("Pivot/Gains/KS(Volts)"), 0),
        pivotKg = Tunable.of(key("Pivot/Gains/KG(Volts)"), 1.48),
        pivotKp = Tunable.of(key("Pivot/Gains/KP"), 10.0),
        pivotCurrentLimit = Tunable.of(key("Pivot/CurrentLimit(Amps)"), 40),
        pivotCurrentZeroVolts = Tunable.of(key("Pivot/CurrentZeroing/Volts"), 2.5),
        pivotCurrentZeroLimit = Tunable.of(key("Pivot/CurrentZeroing/Limit (amps)"), 20.0);
    private final Tunable<Angle>
        stowPos = Tunable.of(key("Positions/Stow"), Radians.of(-100)),
        intakePos = Tunable.of(key("Positions/Intake"), Radians.of(0));

    private PivotHardware pivotIO;
    private RollerHardware rollerIO;
    private final PivotDataAutoLogged pivotInputs = new PivotDataAutoLogged();
    private final RollerDataAutoLogged rollerInputs = new RollerDataAutoLogged();

    private TrapezoidProfile motionProfile;
    @AutoLogOutput private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public GroundIntake() {
        switch (RobotMode.get()) {
            case REAL -> {
                pivotIO = new RelativeNeoIntakePivot();
                rollerIO = new VortexIntakeRollers();
            }
            case REPLAY -> {
                pivotIO = new PivotHardware();
                rollerIO = new RollerHardware();
            }
            case SIM -> {
                var pivotSimConfig = new PivotSimConfig(
                    PIVOT_REDUCTION, PIVOT_MOI, PIVOT_LENGTH,
                    PIVOT_MOTOR_KIND, true
                );
                pivotIO = new SimPivotHardware(pivotSimConfig);
                rollerIO = new SimRollerHardware(DCMotor.getNeoVortex(1), 3.0);
            }
        }
        applyConfigs();
        pivotMaxVel.onChange(this::applyConfigs);
        pivotMaxAccel.onChange(this::applyConfigs);
        pivotCurrentLimit.onChange(this::applyConfigs);
        rollerCurrentLimit.onChange(this::applyConfigs);
    }

    private void applyConfigs() {
        var constraints = new TrapezoidProfile.Constraints(
            pivotMaxVel.get(), pivotMaxAccel.get()
        );
        motionProfile = new TrapezoidProfile(constraints);
        rollerIO.setCurrentLimit(rollerCurrentLimit.get());
        pivotIO.setCurrentLimit(pivotCurrentLimit.get());
        pivotIO.setPDGains(pivotKp.get(), PIVOT_KD);
    }

    private Command setAngleCmd(Supplier<Angle> target) {
        var goal = new TrapezoidProfile.State(0, 0);
        return this.runOnce(() -> setpoint = new TrapezoidProfile.State(pivotInputs.radians, pivotInputs.radiansPerSec))
            .andThen(
                this.run(() -> {
                    goal.position = target.get().in(Radians);
                    setpoint = motionProfile.calculate(0.02, setpoint, goal);
                    double ff = pivotKg.get() * Math.cos(pivotInputs.radians); // pivotInputs.radians must be 0 degrees when horizontal.
                    ff += Math.signum(setpoint.velocity) * pivotKs.get();
                    ff += PIVOT_KV * setpoint.velocity;
                    Logger.recordOutput(key("PivotFF"), ff);
                    pivotIO.setRadians(setpoint.position, ff);
                })
            );
    }

    public Command stowCmd() {
        var cmd = setAngleCmd(stowPos::get)
            .alongWith(Commands.run(() -> rollerIO.setVolts(0)));
        return logged(cmd, "Stow");
    }

    public Command intakeCmd() {
        var cmd = setAngleCmd(intakePos::get)
            .alongWith(Commands.run(() -> rollerIO.setVolts(rollerVolts.get())));
        return logged(cmd, "Intake");
    }

    public Command idleCmd() {
        var cmd = this.run(() -> rollerIO.setVolts(rollerVolts.get()));
        return logged(cmd, "Idle");
    }

    public Command manualPivotCmd(boolean gravityCompensate, DoubleSupplier volts) {
        var cmd = this.run(() -> pivotIO.setVolts(volts.getAsDouble()));
        return logged(cmd, "ManualPivot(gravityCompensate=" + gravityCompensate + ")");
    }

    public Command currentZeroCmd(boolean resetEncoder) {
        var debouncer = new Debouncer(0.2);
        var cmd = this.run(() -> pivotIO.setVolts(pivotCurrentZeroVolts.get()))
            .until(() -> debouncer.calculate(hardStopHit()))
            .finallyDo(wasInterrupted -> {
                pivotIO.setVolts(0);
                if (resetEncoder && !wasInterrupted) pivotIO.zeroEncoder(0);
            });
        return logged(cmd, "PivotCurrentZero");
    }

    private boolean hardStopHit() {
        return pivotInputs.motorStats.appliedAmps() > pivotCurrentZeroLimit.get();
    }

    @Override
    public void loggedPeriodic() {
        pivotIO.refreshData(pivotInputs);
        rollerIO.refreshData(rollerInputs);
        Logger.processInputs(key("Pivot"), pivotInputs);
        Logger.processInputs(key("Rollers"), rollerInputs);
    }
}
