package robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import lib.Convert;
import lib.hardware.MotorStats;
import lib.hardware.SignalRefresh;

public class KrakenFlywheels extends FlywheelHardware {
    private final TalonFX talon = new TalonFX(/*id */ 1);
    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final StatusSignal<AngularVelocity> angularVel = talon.getVelocity();
    private final StatusSignal<Double> pidErr = talon.getClosedLoopError();

    private final TorqueCurrentFOC currentReq = new TorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC velocityReq = new VelocityTorqueCurrentFOC(0);

    public KrakenFlywheels() {
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        talon.getConfigurator().apply(config);
        SignalRefresh.register(100.0, talon.getNetwork(), angularVel, pidErr);
    }

    @Override
    public void setAmps(double amps) {
        talon.setControl(currentReq.withOutput(amps));
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        talon.setControl(velocityReq.withVelocity(velocity));
    }

    @Override
    public void refreshData(FlywheelDataAutoLogged inputs) {
        inputs.velocity = angularVel.getValue();
        inputs.pidError = pidErr.getValueAsDouble();
        inputs.leaderStats = MotorStats.from(talon);
    }

    @Override
    public void setGains(double kP, double kD, double kV) {
        config.Slot0.kP = kP / Convert.RADIANS_TO_ROTATIONS;
        config.Slot0.kD = kD / Convert.RADIANS_TO_ROTATIONS;
        config.Slot0.kV = kV / Convert.RADIANS_TO_ROTATIONS;
        talon.getConfigurator().apply(config);
    }
}
