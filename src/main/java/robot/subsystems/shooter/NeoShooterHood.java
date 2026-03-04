package robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import lib.Convert;
import lib.hardware.MotorStats;
import robot.subsystems.common.PivotDataAutoLogged;
import robot.subsystems.common.PivotHardware;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

/**
 * The hardware powering the shooter hood on the real robot.
 * Responsible for controlling the trajectory of the fuel.
 */
public class NeoShooterHood extends PivotHardware {
    private final SparkMax motor = new SparkMax(31, SparkLowLevel.MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();
    private final SparkClosedLoopController pid = motor.getClosedLoopController();
    private final RelativeEncoder encoder = motor.getEncoder();
    private final Alert configError = new Alert("Intake Pivot failed to configure.", Alert.AlertType.kError);

    public NeoShooterHood() {
        config.encoder
            .positionConversionFactor(1 / Shooter.HOOD_REDUCTION)
            .velocityConversionFactor(1 / Shooter.HOOD_REDUCTION);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder.setPosition(Degrees.of(19).in(Rotations));
    }

    @Override
    public void refreshData(PivotDataAutoLogged data) {
        data.positionRad = encoder.getPosition() * Convert.ROTATIONS_TO_RADIANS;
        data.motorStats = MotorStats.from(motor);
    }

    @Override
    public void setVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setRadians(double radians, double feedforwardV) {
        pid.setSetpoint(
            radians * Convert.RADIANS_TO_ROTATIONS,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforwardV
        );
    }

    @Override
    public void setPDGains(double p, double d) {
        config.closedLoop.pid(p / Convert.RADIANS_TO_ROTATIONS, 0, d / Convert.RADIANS_TO_DEGREES, ClosedLoopSlot.kSlot0);
        configureMotor();
    }

    @Override
    public void setCurrentLimit(double amps) {
        config.smartCurrentLimit((int) amps);
        configureMotor();
    }

    private void configureMotor() {
        var status = motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        configError.set(status != REVLibError.kError);
    }
}

