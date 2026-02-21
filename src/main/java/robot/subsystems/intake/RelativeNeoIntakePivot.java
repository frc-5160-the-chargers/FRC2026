package robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkFlexConfig;
import lib.Convert;
import lib.hardware.MotorStats;
import robot.subsystems.common.PivotDataAutoLogged;
import robot.subsystems.common.PivotHardware;

/**
 * The hardware powering the intake pivot on the real robot,
 * using the neo's onboard relative encoder.
 */
public class RelativeNeoIntakePivot extends PivotHardware {
    private final SparkMax motor = new SparkMax(15, SparkLowLevel.MotorType.kBrushless);
    private final SparkFlexConfig config = new SparkFlexConfig();
    private final SparkClosedLoopController pid = motor.getClosedLoopController();
    private final RelativeEncoder encoder = motor.getEncoder();

    public RelativeNeoIntakePivot() {
        config.encoder
            .positionConversionFactor(1 / GroundIntake.PIVOT_REDUCTION)
            .velocityConversionFactor(1 / GroundIntake.PIVOT_REDUCTION);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void refreshData(PivotDataAutoLogged data) {
        data.radians = encoder.getPosition() * Convert.ROTATIONS_TO_RADIANS;
        data.radiansPerSec = encoder.getVelocity() * Convert.RPM_TO_RADIANS_PER_SECOND;
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
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setCurrentLimit(double amps) {
        config.smartCurrentLimit((int) amps);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void zeroEncoder(double radians) {
        encoder.setPosition(radians * Convert.RADIANS_TO_ROTATIONS);
    }
}

