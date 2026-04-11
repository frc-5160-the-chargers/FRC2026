package robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lib.commands.CmdSequence;
import org.littletonrobotics.junction.AutoLogOutput;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.*;

public class OperatorController extends CommandXboxController implements Subsystem {
    public OperatorController(int port) {
        super(port);
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
        return this.run(() -> setRumble(kRightRumble, 0.35))
            .withTimeout(1.0)
            .finallyDo(() -> setRumble(kRightRumble, 0.0))
            .withName("Operator#NotifySerializerReady");
    }

    public Command notifyOutOfBoundsCmd() {
        return CmdSequence.of(
            this.run(() -> setRumble(kLeftRumble, 1.0))
                .withTimeout(0.5),
            this.run(() -> setRumble(kBothRumble, 0.0))
                .withTimeout(0.2)
        )
            .repeatedly()
            .withName("Operator#NotifyOutOfBounds");
    }
}
