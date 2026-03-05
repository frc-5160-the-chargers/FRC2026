package robot.subsystems.serializer;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import lib.Convert;
import lib.hardware.MotorStats;
import lib.hardware.SignalRefresh;
import robot.subsystems.common.RollerDataAutoLogged;
import robot.subsystems.common.RollerHardware;

/** The hardware powering the serializer motors on the real robot. */
public class KrakenSerializerRollers extends RollerHardware {
    private final TalonFX motor = new TalonFX(7);
    private final StatusSignal<AngularVelocity> angularVel = motor.getVelocity();
    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final VoltageOut controlReq = new VoltageOut(0);

    public KrakenSerializerRollers() {
        setCurrentLimit(Serializer.CURRENT_LIMIT);
        setCoastMode(false);
        SignalRefresh.register(100.0, motor.getNetwork(), angularVel);
        motor.optimizeBusUtilization();
    }

    @Override
    public void refreshData(RollerDataAutoLogged inputs) {
        inputs.motorStats = new MotorStats[] {MotorStats.from(motor)};
        inputs.velocityRadPerSec = angularVel.getValueAsDouble() * Convert.ROTATIONS_TO_RADIANS;
    }

    @Override
    public void setVolts(double volts) {
        motor.setControl(controlReq.withOutput(volts));
    }

    @Override
    public void setCurrentLimit(double amps) {
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        motor.getConfigurator().apply(config);
    }

    @Override
    public void setCoastMode(boolean enabled) {
        config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Coast: NeutralModeValue.Brake;
        motor.getConfigurator().apply(config);
    }
}
