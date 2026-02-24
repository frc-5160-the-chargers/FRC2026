package robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.Convert;
import lib.RobotMode;
import lib.Tunable;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.ChargerSubsystem;
import robot.subsystems.common.*;
import robot.subsystems.common.PivotController.PivotConstraints;
import robot.subsystems.common.PivotController.PivotGains;
import robot.subsystems.common.PivotHardware.PivotSimConfig;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static lib.Convert.CustomUnits.PoundSquareInches;

@SuppressWarnings("FieldCanBeLocal")
public class GroundIntake extends ChargerSubsystem {
    public static final Distance PIVOT_LENGTH = Inches.of(12.4);
    static final MomentOfInertia PIVOT_MOI = PoundSquareInches.of(1344.71);
    static final double PIVOT_REDUCTION = 25.0 * 24.0 / 22.0; // MaxPlanetary + Sprockets
    static final double ROLLER_REDUCTION = 1.0;
    static final double PIVOT_OFFSET_ROTATIONS = -0.2225;

    static final DCMotor ROLLER_MOTOR_KIND = DCMotor.getNeoVortex(1);
    static final DCMotor PIVOT_MOTOR_KIND = DCMotor.getNEO(1);
    static final double PIVOT_KV = 1 / PIVOT_MOTOR_KIND.withReduction(PIVOT_REDUCTION).KvRadPerSecPerVolt;

    static final PivotConstraints CONSTRAINTS = new PivotConstraints(4, 5);
    static final PivotGains GAINS = new PivotGains(0.03, PIVOT_KV, -0.38, 5.0, 0);

    private final Tunable<Double>
        rollerVolts = Tunable.of(key("Rollers/Power(Volts)"), 3.5),
        rollerCurrentLimit = Tunable.of(key("Rollers/CurrentLimit(Amps)"), 55),
        pivotCurrentLimit = Tunable.of(key("Pivot/CurrentLimit(Amps)"), 40),
        pivotCurrentZeroVolts = Tunable.of(key("Pivot/CurrentZeroing/Volts"), 2.5),
        pivotCurrentZeroLimit = Tunable.of(key("Pivot/CurrentZeroing/Limit (amps)"), 20.0);
    private final Tunable<Angle>
        stowPos = Tunable.of(key("Positions/Stow"), Degrees.of(-140)),
        intakePos = Tunable.of(key("Positions/Intake"), Radians.of(-0.25));

    private PivotHardware pivotIO;
    private RollerHardware rollerIO;
    private final PivotDataAutoLogged pivotInputs = new PivotDataAutoLogged();
    private final RollerDataAutoLogged rollerInputs = new RollerDataAutoLogged();

    private final PivotController pivotController;

    public GroundIntake() {
        switch (RobotMode.get()) {
            case REAL -> {
                pivotIO = new NeoIntakePivot();
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
                pivotIO.zeroEncoder(-170 * Convert.DEGREES_TO_RADIANS);
                rollerIO = new SimRollerHardware(DCMotor.getNeoVortex(1), 3.0);
            }
        }
        pivotController = new PivotController(key("Pivot"), GAINS, CONSTRAINTS, pivotIO);
        pivotCurrentLimit.onChange(pivotIO::setCurrentLimit);
        rollerCurrentLimit.onChange(rollerIO::setCurrentLimit);
    }

    public Command manualRollersCmd(DoubleSupplier output) {
        var cmd = this.run(() -> {
            rollerIO.setVolts(output.getAsDouble());
            pivotIO.setVolts(0);
        });
        return logged(cmd, "ManualRollers");
    }

    private Command setAngleCmd(Supplier<Angle> target) {
        var resetStateCmd = this.runOnce(() -> pivotController.resetTo(pivotInputs.getMotionState()));
        var targetAngleCmd = this.run(() -> {
            var goal = new TrapezoidProfile.State(target.get().in(Radians), 0);
            pivotController.moveTo(goal, pivotInputs.positionRad);
        });
        return resetStateCmd.andThen(targetAngleCmd);
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
        var cmd = this.run(() -> {
            double ff = gravityCompensate ? pivotController.getGravityComp(pivotInputs.positionRad) : 0;
            pivotIO.setVolts(volts.getAsDouble() + ff);
            rollerIO.setVolts(0);
        });
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
