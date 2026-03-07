package robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.AutoLogOutput;

public class ManualOverrideController extends CommandXboxController {
    public ManualOverrideController(int port) {
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
}
