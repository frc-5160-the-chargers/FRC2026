package robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import lib.Convert;
import lib.hardware.MotorStats;
import robot.subsystems.common.RollerDataAutoLogged;
import robot.subsystems.common.RollerHardware;

/** The hardware powering the intake rollers on the real robot. */
public class VortexIntakeRollers extends RollerHardware {
    private final SparkMax motor = new SparkMax(7, SparkLowLevel.MotorType.kBrushless);
    private final SparkFlexConfig config = new SparkFlexConfig();
    private final RelativeEncoder encoder = motor.getEncoder();

    public VortexIntakeRollers() {
        config.encoder
            .positionConversionFactor(1 / GroundIntake.ROLLER_REDUCTION)
            .velocityConversionFactor(1 / GroundIntake.ROLLER_REDUCTION);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void refreshData(RollerDataAutoLogged data) {
        data.radiansPerSec = encoder.getVelocity() * Convert.RPM_TO_RADIANS_PER_SECOND;
        data.motorStats = new MotorStats[]{MotorStats.from(motor)};
    }

    @Override
    public void setVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setCurrentLimit(double amps) {
        config.smartCurrentLimit((int) amps);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
