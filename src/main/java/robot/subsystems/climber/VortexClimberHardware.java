package robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import lib.Convert;

public class VortexClimberHardware extends ClimberHardware {
    private final SparkFlex motor = new SparkFlex(1, MotorType.kBrushless);
    private final SparkFlexConfig config = new SparkFlexConfig();
    private final SparkClosedLoopController pid = motor.getClosedLoopController();
    private final RelativeEncoder encoder = motor.getEncoder();

    
    // config.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    public VortexClimberHardware() {
        configureMotor();
    }

    @Override
    public void refreshData(ClimberData data) {
        data.radians = encoder.getPosition() * Convert.ROTATIONS_TO_RADIANS;
        data.volts = motor.getBusVoltage() * motor.getAppliedOutput();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setRadians(double radians) {
        pid.setSetpoint(radians * Convert.RADIANS_TO_ROTATIONS, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void configureMotor() {
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setPDGains(double p, double d) {
        config.closedLoop.pid(p / Convert.RADIANS_TO_ROTATIONS, 0, d / Convert.RADIANS_TO_DEGREES, ClosedLoopSlot.kSlot0);
    }
}