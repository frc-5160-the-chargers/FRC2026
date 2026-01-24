package robot.subsystems.shooter.flywheels;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class KrakenFlywheelHardware extends FlywheelHardware {
    private final TalonFX talon = new TalonFX(/*id */ 1);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    @Override
    public void setVoltage(double volts) {
        talon.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setVelocity(double radiansPerSecond) {
        super.setVelocity(radiansPerSecond);
    }
}
