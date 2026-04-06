package robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import lib.Convert;
import lib.RobotMode;
import lib.hardware.MotorStats;
import lib.hardware.SignalRefresh;
import org.littletonrobotics.junction.AutoLog;
import robot.constants.RobotConfig;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

// We opt to use hardware injection sim for the flywheels
// to make it easier to use torque current control.
public class KrakenFlywheels {
    @AutoLog
    static class FlywheelData {
        public AngularVelocity velocity = RadiansPerSecond.zero();
        public AngularAcceleration accel = RadiansPerSecondPerSecond.zero();
        public MotorStats motorStats = MotorStats.EMPTY;
    }

    private final TalonFX talon = new TalonFX(/*id */ 16, RobotConfig.CANIVORE);
    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final StatusSignal<AngularVelocity> velocity = talon.getVelocity();
    private final StatusSignal<AngularAcceleration> accel = talon.getAcceleration();

    private final CoastOut idleReq = new CoastOut();
    private final TorqueCurrentFOC currentReq = new TorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC velocityReq = new VelocityTorqueCurrentFOC(0);

    private final Alert configError = new Alert("Kraken Flywheels failed to configure.", Alert.AlertType.kError);

    private final DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 0.003268, Shooter.FLYWHEEL_REDUCTION
        ),
        DCMotor.getKrakenX60(1)
    );

    public KrakenFlywheels() {
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = Shooter.FLYWHEEL_REDUCTION;
        config.Feedback.VelocityFilterTimeConstant = 0.1;
        var status = talon.getConfigurator().apply(config);
        configError.set(!status.isOK());
        SignalRefresh.register(100.0, talon.getNetwork(), velocity, accel);
        talon.optimizeBusUtilization();
        // CTRE's hardware injection sim mostly relies on itself being run
        // at a higher frequency to be less innacurate.
        if (RobotMode.isSim()) new Notifier(this::updateSim).startPeriodic(0.005);
    }

    public void setCoast() {
        talon.setControl(idleReq);
    }

    public void setAmps(double amps) {
        talon.setControl(currentReq.withOutput(amps));
    }

    public void setVelocity(AngularVelocity velocity) {
        talon.setControl(velocityReq.withVelocity(velocity));
    }

    public void refreshData(FlywheelDataAutoLogged inputs) {
        inputs.velocity = velocity.getValue();
        inputs.accel = accel.getValue();
        inputs.motorStats = MotorStats.from(talon);
    }

    public void setGains(double kP, double kD, double kV) {
        config.Slot0.kP = kP / Convert.RADIANS_TO_ROTATIONS;
        config.Slot0.kD = kD / Convert.RADIANS_TO_ROTATIONS;
        config.Slot0.kV = kV / Convert.RADIANS_TO_ROTATIONS;
        var status = talon.getConfigurator().apply(config.Slot0);
        configError.set(!status.isOK());
    }

    public void setCurrentLimit(double amps) {
        config.TorqueCurrent.PeakForwardTorqueCurrent = amps;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -amps;
        var status = talon.getConfigurator().apply(config.TorqueCurrent);
        configError.set(!status.isOK());
    }

    private void updateSim() {
        sim.update(0.005);
        sim.setInputVoltage(talon.getSimState().getMotorVoltage());
        talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        talon.getSimState().setRawRotorPosition(sim.getAngularPosition().div(Shooter.FLYWHEEL_REDUCTION));
        talon.getSimState().setRotorVelocity(sim.getAngularVelocity().div(Shooter.FLYWHEEL_REDUCTION));
    }
}
