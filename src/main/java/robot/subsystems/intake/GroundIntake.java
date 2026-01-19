package robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.RobotMode;
import lib.Tunable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.ChargerSubsystem;
import robot.subsystems.common.RollerHardware;
import robot.subsystems.common.SimRollerHardware;
import robot.subsystems.pivot.PivotDataAutoLogged;
import robot.subsystems.common.PivotHardware;
import robot.subsystems.common.PivotHardware.PivotSimConfig;
import robot.subsystems.common.SimPivotHardware;
import robot.subsystems.rollers.RollerDataAutoLogged;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class GroundIntake extends ChargerSubsystem {
    private final Tunable<Double>
        maxVelRadPerSec = Tunable.of(key("Pivot/MaxVel(rad per s)"), 5),
        maxAccelRadPerSecSq = Tunable.of(key("Pivot/MaxAccel(rad per s^2)"), 10),
        intakeVolts = Tunable.of(key("Rollers/IntakeVolts"), 6);
    private final Tunable<Angle>
        stowPos = Tunable.of(key("Positions/Stow"), Radians.of(0)),
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
                pivotIO = new IntakePivotHardware();
                rollerIO = new IntakeRollersHardware();
            }
            case REPLAY -> {
                pivotIO = new PivotHardware();
                rollerIO = new RollerHardware();
            }
            case SIM -> {
                var pivotSimConfig = new PivotSimConfig( // TODO find actual values
                    6.0, KilogramSquareMeters.of(0.025),
                    60.0, Meters.of(0.2),
                    DCMotor.getNEO(1), true
                );
                pivotIO = new SimPivotHardware(pivotSimConfig);
                rollerIO = new SimRollerHardware(
                    DCMotor.getNeoVortex(1),
                    KilogramSquareMeters.of(0.001),
                    3.0
                );
            }
        }
        initMotionProfile();
        maxVelRadPerSec.onChange(this::initMotionProfile);
        maxAccelRadPerSecSq.onChange(this::initMotionProfile);
    }

    private void initMotionProfile() {
        var constraints = new TrapezoidProfile.Constraints(
            maxVelRadPerSec.get(), maxAccelRadPerSecSq.get()
        );
        motionProfile = new TrapezoidProfile(constraints);
    }

    private Command setAngleCmd(Supplier<Angle> target) {
        var goal = new TrapezoidProfile.State(0, 0);
        return this.run(() -> {
            goal.position = target.get().in(Radians);
            setpoint = motionProfile.calculate(0.02, setpoint, goal);
            pivotIO.setRadians(setpoint.position, 0);
        });
    }

    public Command stowCmd() {
        return setAngleCmd(stowPos::get)
            .alongWith(Commands.run(() -> rollerIO.setVolts(0)))
            .withName("GroundIntakeStow");
    }

    public Command intakeCmd() {
        return setAngleCmd(intakePos::get)
            .alongWith(Commands.run(() -> rollerIO.setVolts(intakeVolts.get())))
            .withName("IntakePivotIntake");
    }

    @Override
    public void loggedPeriodic() {
        pivotIO.refreshData(pivotInputs);
        rollerIO.refreshData(rollerInputs);
        Logger.processInputs(key("Pivot"), pivotInputs);
        Logger.processInputs(key("Rollers"), rollerInputs);
    }
}
