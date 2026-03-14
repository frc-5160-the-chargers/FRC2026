package robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.AutoLogOutput;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kLeftRumble;
import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kRightRumble;

// implementing Subsystem allows you to use ChargerSubsystem's functionalities
// while inheriting from another class. However, you use a lot of logging utilities
// as a result, and must call register() manually in the constructor of your class.
public class OperatorController extends CommandXboxController implements Subsystem {
    public OperatorController(int port) {
        super(port);
        register();
    }

    @AutoLogOutput
    public double getManualPivotVolts() {
        return MathUtil.applyDeadband(getLeftY(), 0.2) * 2;
    }

    @AutoLogOutput
    public double getFlywheelSpeedAdjustment() {
        return MathUtil.applyDeadband(getRightY(), 0.1) / 2.0 + 1.0;
    }

    public Command notifySerializerReadyCmd() {
        return this.run(() -> setRumble(kRightRumble, 0.2))
            .withTimeout(1.0)
            .finallyDo(() -> setRumble(kRightRumble, 0.0))
            .withName("Operator#NotifySerializerReady");
    }

    public Command notifyHubShiftCmd() {
        return this.run(() -> setRumble(kLeftRumble, 0.2))
            .withTimeout(0.5)
            .finallyDo(() -> setRumble(kLeftRumble, 0.0))
            .withName("Driver#NotifySerializerReady");
    }
}
