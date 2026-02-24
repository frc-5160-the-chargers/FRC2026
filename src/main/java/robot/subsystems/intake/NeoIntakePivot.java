package robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.Convert;
import lib.hardware.MotorStats;
import org.littletonrobotics.junction.AutoLogOutput;
import robot.subsystems.common.PivotDataAutoLogged;
import robot.subsystems.common.PivotHardware;

/**
 * The hardware powering the intake pivot on the real robot,
 * using a duty-cycle encoder to zero the neo's onboard relative encoder.
 */
public class NeoIntakePivot extends PivotHardware {
    private final SparkMax motor = new SparkMax(15, SparkLowLevel.MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();
    private final SparkClosedLoopController pid = motor.getClosedLoopController();
    private final RelativeEncoder encoder = motor.getEncoder();
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(3); // TODO set ID

    public NeoIntakePivot() {
        config.encoder
            .positionConversionFactor(1 / GroundIntake.PIVOT_REDUCTION)
            .velocityConversionFactor(1 / GroundIntake.PIVOT_REDUCTION);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        var initCmd = Commands.waitSeconds(5)
            .andThen(() -> encoder.setPosition(getAbsoluteRot()))
            .ignoringDisable(true)
            .withName("Pivot Encoder Zeroing");
        CommandScheduler.getInstance().schedule(initCmd);
    }

    @AutoLogOutput(key = "GroundIntake/Pivot/AbsoluteRotations")
    private double getAbsoluteRot() {
        return absoluteEncoder.get() + GroundIntake.PIVOT_OFFSET_ROTATIONS;
    }

    @Override
    public void refreshData(PivotDataAutoLogged data) {
        data.positionRad = encoder.getPosition() * Convert.ROTATIONS_TO_RADIANS;
        data.velocityRadPerSec = encoder.getVelocity() * Convert.RPM_TO_RADIANS_PER_SECOND;
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

