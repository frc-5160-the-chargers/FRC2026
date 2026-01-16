package robot.subsystems.pivot.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import lib.RobotMode;
import lib.Tunable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.ChargerSubsystem;
import robot.subsystems.pivot.PivotDataAutoLogged;
import robot.subsystems.pivot.PivotHardware;
import robot.subsystems.pivot.PivotHardware.PivotHardwareCfg;
import robot.subsystems.pivot.SimPivotHardware;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class IntakePivot extends ChargerSubsystem {
    private final Tunable<Double>
        maxVelRadPerSec = Tunable.of(key("MaxVel(rad per s)"), 5),
        maxAccelRadPerSecSq = Tunable.of(key("MaxAccel(rad per s^2)"), 10);
    private final Tunable<Angle>
        stowPos = Tunable.of(key("Positions/Stow"), Radians.of(0)),
        intakePos = Tunable.of(key("Positions/Intake"), Radians.of(0));

    private final PivotDataAutoLogged inputs = new PivotDataAutoLogged();
    private final PivotHardware io = switch (RobotMode.get()) {
        case REAL, REPLAY -> new PivotHardware();
        case SIM -> new SimPivotHardware(
            new PivotHardwareCfg(
                6.0, KilogramSquareMeters.of(0.025),
                60.0, Meters.of(0.2),
                DCMotor.getNEO(1), true
            ) // TODO find actual values
        );
    };
    private TrapezoidProfile motionProfile;
    @AutoLogOutput private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public IntakePivot() {
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
            io.setRadians(setpoint.position, 0);
        });
    }

    public Command stowCmd() {
        return setAngleCmd(stowPos::get).withName("IntakePivotStow");
    }

    public Command intakeCmd() {
        return setAngleCmd(intakePos::get).withName("IntakePivotIntake");
    }

    @Override
    public void loggedPeriodic() {
        io.refreshData(inputs);
        Logger.processInputs("IntakePivot", inputs);
    }
}
