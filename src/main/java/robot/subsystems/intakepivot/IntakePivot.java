package robot.subsystems.intakepivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import lib.RobotMode;
import lib.Tunable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.ChargerSubsystem;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Radians;

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
        case SIM -> new SimPivotHardware();
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
