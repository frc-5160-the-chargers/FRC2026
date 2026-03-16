package robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.Command;
import lib.Convert;
import lib.RobotMode;
import lib.Tunable;
import lib.commands.CmdSequence;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.ChargerSubsystem;
import robot.subsystems.common.*;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static lib.Convert.CustomUnits.PoundSquareInches;

@SuppressWarnings("FieldCanBeLocal")
public class GroundIntake extends ChargerSubsystem {
    static final double PIVOT_REDUCTION = 25.0 * 24.0 / 22.0; // MaxPlanetary + Sprockets
    static final Angle PIVOT_OFFSET = Radians.of(5.28 - 6.28);
    static final Distance PIVOT_LENGTH = Inches.of(12.4);
    static final MomentOfInertia PIVOT_MOI = PoundSquareInches.of(1344.71);
    static final DCMotor PIVOT_MOTOR_KIND = DCMotor.getNEO(1);
    static final double PIVOT_KV = 1 / PIVOT_MOTOR_KIND.withReduction(PIVOT_REDUCTION).KvRadPerSecPerVolt;

    static final DCMotor ROLLER_MOTOR_KIND = DCMotor.getNeoVortex(1);
    static final double ROLLER_REDUCTION = 1.0;
    static final double ROLLER_KV = 1 / ROLLER_MOTOR_KIND.withReduction(ROLLER_REDUCTION).KvRadPerSecPerVolt;
    static final Distance ROLLER_WHEEL_RADIUS = Inches.of(2);

    private final Tunable<Double>
        rollerReverseVolts = Tunable.of(key("Rollers/ReverseVolts"), 6.0),
        rollerCurrentLimit = Tunable.of(key("Rollers/CurrentLimit"), 55),
        rollerTargetVel = Tunable.of(key("Rollers/ClosedLoopRadPerSec"), 150),
        rollerKp = Tunable.of(key("Rollers/ClosedLoopKp"), 0.02);
    private final Tunable<Double>
        pivotCurrentLimit = Tunable.of(key("Pivot/CurrentLimit(Amps)"), 40),
        pivotCurrentZeroVolts = Tunable.of(key("Pivot/CurrentZeroing/Volts"), 2.5),
        pivotCurrentZeroLimit = Tunable.of(key("Pivot/CurrentZeroing/Limit (amps)"), 20.0);
    private final Tunable<Double>
        pivotMaxVel = Tunable.of(key("Pivot/MaxVel(rad per s)"), 7.0),
        pivotMaxAccel = Tunable.of(key("Pivot/MaxAccel(rad per s^2)"), 7.0),
        pivotKs = Tunable.of(key("Pivot/Gains/KS(Volts)"), 0.03),
        pivotKg = Tunable.of(key("Pivot/Gains/KG(Volts)"), -0.38),
        pivotKp = Tunable.of(key("Pivot/Gains/KP"), 5.0);
    private final Tunable<Angle>
        stowPos = Tunable.of(key("Positions/Stow"), Degrees.of(-120)),
        intakePos = Tunable.of(key("Positions/Intake"), Degrees.of(-4));

    private IntakePivot pivotIO;
    private RollerHardware rollerIO;
    private final PivotDataAutoLogged pivotInputs = new PivotDataAutoLogged();
    private final RollerDataAutoLogged rollerInputs = new RollerDataAutoLogged();

    private final Debouncer hardStopDebouncer = new Debouncer(0.2);
    private final Optional<IntakeSimulation> sim;

    private TrapezoidProfile motionProfile;
    @AutoLogOutput private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public GroundIntake(Optional<IntakeSimulation> sim) {
        switch (RobotMode.get()) {
            case REAL -> {
                pivotIO = new NeoIntakePivot();
                rollerIO = new VortexIntakeRollers();
            }
            case REPLAY -> {
                pivotIO = new IntakePivot();
                rollerIO = new RollerHardware();
            }
            case SIM -> {
                pivotIO = new SimIntakePivot();
                rollerIO = new SimRollerHardware(ROLLER_MOTOR_KIND, ROLLER_REDUCTION);
                pivotIO.zeroEncoder(-170 * Convert.DEGREES_TO_RADIANS);
            }
        }
        this.sim = sim;
        applyConfigs();
        pivotCurrentLimit.onChange(this::applyConfigs);
        rollerCurrentLimit.onChange(this::applyConfigs);
        pivotMaxVel.onChange(this::applyConfigs);
        pivotMaxAccel.onChange(this::applyConfigs);
        pivotKp.onChange(this::applyConfigs);
    }

    private void applyConfigs() {
        var constraints = new TrapezoidProfile.Constraints(pivotMaxVel.get(), pivotMaxAccel.get());
        motionProfile = new TrapezoidProfile(constraints);
        pivotIO.setPDGains(pivotKp.get(), 0.0);
        pivotIO.setCurrentLimit(pivotCurrentLimit.get());
        rollerIO.setCurrentLimit(rollerCurrentLimit.get());
    }

    @AutoLogOutput
    private boolean atHardStop() {
        return pivotInputs.positionRad < -100 * Convert.DEGREES_TO_RADIANS;
    }

    private void setRollerVolts(double volts) {
        rollerIO.setVolts(volts);
        if (sim.isEmpty()) return;
        if (volts > 0.05) {
            sim.get().startIntake();
        } else {
            sim.get().stopIntake();
        }
    }

    private void setPivotPosition(Angle angle) {
        var goal = new TrapezoidProfile.State(angle.in(Radians), 0);
        setpoint = motionProfile.calculate(0.02, setpoint, goal);
        double ff = pivotKg.get() * Math.cos(pivotInputs.positionRad); // pivotInputs.radians must be 0 degrees when horizontal.
        ff += Math.signum(setpoint.velocity) * pivotKs.get();
        ff += PIVOT_KV * setpoint.velocity;
        Logger.recordOutput(key("PivotFF"), ff);
        pivotIO.setRadians(setpoint.position, ff);
    }

    /**
     * A command that moves the intake to its stow position,
     * with a little bit of voltage to push away fuel.
     */
    public Command stowCmd() {
        var cmd = CmdSequence.of(
            this.runOnce(() -> setpoint = pivotInputs.getMotionState()),
            this.run(() -> {
                setPivotPosition(stowPos.get());
                setRollerVolts(1.0);
            })
        );
        return logged(cmd, "Stow");
    }

    /**
     * A command that runs the intake and deploys the pivot, with scaling based off the robot's speed.
     * @param robotSpeeds The robot-relative chassis speeds of the robot.
     * @param intakeSpeedMultiplier A multiplier for intaking speed.
     */
    public Command deployCmd(double intakeSpeedMultiplier, Supplier<ChassisSpeeds> robotSpeeds) {
        var cmd = CmdSequence.of(
            this.runOnce(() -> setpoint = pivotInputs.getMotionState()),
            this.run(() -> {
                setPivotPosition(intakePos.get());
                var vx = robotSpeeds.get().vxMetersPerSecond;
                var targetVel = rollerTargetVel.get() + Math.abs(vx) / ROLLER_WHEEL_RADIUS.in(Meters);
                targetVel *= intakeSpeedMultiplier;
                var velocityErr = targetVel - rollerInputs.velocityRadPerSec;
                setRollerVolts(atHardStop() ? 0 : (rollerKp.get() * velocityErr + ROLLER_KV * targetVel));
            })
        );
        return logged(cmd, "Deploy & Run (Velocity-Based)");
    }

    public Command outtakeCmd() {
        return logged(this.run(() -> rollerIO.setVolts(rollerReverseVolts.get())), "Outtake");
    }

    /** Alternates between the deploy and stow positions to agitate fuel. */
    public Command agitateCmd() {
        var zeroSpeed = new ChassisSpeeds();
        var cmd = CmdSequence.of(
            stowCmd().withTimeout(2.0),
            deployCmd(1.0, () -> zeroSpeed).withTimeout(2.0)
        )
            .repeatedly();
        return logged(cmd, "Agitate");
    }

    /** Runs manual control on the pivot and the intake. */
    public Command manualCmd(DoubleSupplier volts, BooleanSupplier shouldRunIntake) {
        var cmd = this.run(() -> {
            double antiGravityVolts = pivotKg.get() * Math.cos(pivotInputs.positionRad);
            pivotIO.setVolts(volts.getAsDouble() + antiGravityVolts);
            setRollerVolts(shouldRunIntake.getAsBoolean() ? 2.5 : 0);
        });
        return logged(cmd, "ManualPivot");
    }

    // TODO maybe use this for zeroing instead
    public Command currentZeroCmd(boolean resetEncoder) {
        var cmd = this.run(() -> pivotIO.setVolts(pivotCurrentZeroVolts.get()))
            .until(this::hardStopHit)
            .finallyDo(wasInterrupted -> {
                pivotIO.setVolts(0);
                if (resetEncoder && !wasInterrupted) pivotIO.zeroEncoder(0);
            });
        return logged(cmd, "PivotCurrentZero");
    }

    @AutoLogOutput
    private boolean hardStopHit() {
        boolean res = Math.abs(pivotInputs.motorStats.appliedAmps()) > pivotCurrentZeroLimit.get();
        return hardStopDebouncer.calculate(res);
    }

    @Override
    public void loggedPeriodic() {
        pivotIO.refreshData(pivotInputs);
        rollerIO.refreshData(rollerInputs);
        Logger.processInputs(key("Pivot"), pivotInputs);
        Logger.processInputs(key("Rollers"), rollerInputs);
    }
}
