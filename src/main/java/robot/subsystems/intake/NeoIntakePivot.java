package robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.Convert;
import lib.hardware.MotorStats;
import org.littletonrobotics.junction.AutoLogOutput;
import robot.subsystems.common.PivotDataAutoLogged;
import robot.subsystems.common.PivotHardware;

import static edu.wpi.first.units.Units.Rotations;

/**
 * The hardware powering the intake pivot on the real robot,
 * using a duty-cycle encoder to zero the neo's onboard relative encoder.
 */
public class NeoIntakePivot extends PivotHardware {
    private final SparkMax motor = new SparkMax(15, SparkLowLevel.MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();
    private final SparkClosedLoopController pid = motor.getClosedLoopController();
    private final RelativeEncoder encoder = motor.getEncoder();
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(3);
    private final Alert configError = new Alert("Intake Pivot failed to configure.", Alert.AlertType.kError);

    public NeoIntakePivot() {
        config.encoder
            .positionConversionFactor(1 / GroundIntake.PIVOT_REDUCTION)
            .velocityConversionFactor(1 / GroundIntake.PIVOT_REDUCTION);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        configureMotor();
        var initCmd = Commands.waitSeconds(5)
            .andThen(() -> encoder.setPosition(getAbsoluteRot()))
            .ignoringDisable(true)
            .withName("Pivot Encoder Zeroing");
        CommandScheduler.getInstance().schedule(initCmd);
    }

    @AutoLogOutput(key = "GroundIntake/Pivot/AbsoluteRotations")
    private double getAbsoluteRot() {
        return -(absoluteEncoder.get() + GroundIntake.PIVOT_OFFSET.in(Rotations));
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

    @Override
    public void zeroEncoder(double radians) {
        encoder.setPosition(radians * Convert.RADIANS_TO_ROTATIONS);
    }
}

