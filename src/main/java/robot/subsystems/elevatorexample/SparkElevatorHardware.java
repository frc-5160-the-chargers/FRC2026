package robot.subsystems.elevatorexample;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Alert;
import lib.Convert;
import lib.hardware.MotorStats;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.wpilibj.Alert.AlertType.kError;
import static robot.subsystems.elevatorexample.ElevatorConsts.*;

public class SparkElevatorHardware extends ElevatorHardware {
    private static final SparkBaseConfig leaderConfig =
        new SparkMaxConfig()
            .smartCurrentLimit(CURRENT_LIMIT)
            .secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT)
            .inverted(INVERTED)
            .idleMode(SparkBaseConfig.IdleMode.kBrake);
    private static final SparkBaseConfig followerConfig =
        new SparkMaxConfig()
            .smartCurrentLimit(CURRENT_LIMIT)
            .secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .follow(LEADER_MOTOR_ID, true);

    private final SparkMax leader = new SparkMax(LEADER_MOTOR_ID, kBrushless);
    private final RelativeEncoder encoder = leader.getEncoder();
    private final SparkClosedLoopController pid = leader.getClosedLoopController();
    private final SparkMax follower = new SparkMax(FOLLOWER_MOTOR_ID, kBrushless);

    public SparkElevatorHardware() {
        configureLeader();
        var followConfigRes = follower.configure(followerConfig, kResetSafeParameters, kPersistParameters);
        new Alert("Elevator Follower motor didn't confibgure", kError)
            .set(followConfigRes != REVLibError.kOk);
    }

    private void configureLeader() {
        var leaderConfigRes = leader.configure(leaderConfig, kResetSafeParameters, kPersistParameters);
        new Alert("Elevator Leader motor didn't configure", kError)
            .set(leaderConfigRes != REVLibError.kOk);
    }

    @Override
    public void setPDGains(double p, double d) {
        leaderConfig.closedLoop.pid(p, 0, d, ClosedLoopSlot.kSlot0);
        configureLeader();
    }

    @Override
    public void refreshData(ElevatorDataAutoLogged data) {
        data.radians = encoder.getPosition() * Convert.ROTATIONS_TO_RADIANS;
        data.radiansPerSec = encoder.getVelocity() * Convert.RPM_TO_RADIANS_PER_SECOND;
        data.leaderStats = MotorStats.from(leader);
        data.followerStats = MotorStats.from(follower);
    }

    @Override
    public void setRadians(double radians, double feedforwardV) {
        pid.setSetpoint(
            radians / (2 * Math.PI),
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforwardV
        );
    }

    @Override
    public void setVolts(double volts) {
        leader.setVoltage(volts);
    }

    @Override
    public void zeroEncoder() {
        var res = encoder.setPosition(0);
        new Alert("Elevator encoder didn't zero", kError).set(res != REVLibError.kOk);
    }
}