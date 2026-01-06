package robot.subsystems.elevatorexample;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import lib.Convert;
import lib.hardware.MotorStats;
import lib.hardware.SignalRefresh;

public class TalonElevatorHardware extends ElevatorHardware {
    private final TalonFX
        leader = new TalonFX(6),
        follower = new TalonFX(7);
    private final BaseStatusSignal
        position = leader.getPosition(),
        velocity = leader.getVelocity();
    private final PositionVoltage pidReq = new PositionVoltage(0);

    public TalonElevatorHardware() {
        SignalRefresh.register(100, leader.getNetwork(), position, velocity);
        follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    @Override
    public void refreshData(ElevatorDataAutoLogged data) {
        data.radians = position.getValueAsDouble() * Convert.ROTATIONS_TO_RADIANS;
        data.radiansPerSec = velocity.getValueAsDouble() * Convert.ROTATIONS_TO_RADIANS;
        data.leaderStats = MotorStats.from(leader);
        data.followerStats = MotorStats.from(follower);
    }

    @Override
    public void setRadians(double radians, double feedforwardV) {
        leader.setControl(pidReq.withPosition(radians).withFeedForward(feedforwardV));
    }

    @Override
    public void setVolts(double volts) {
        leader.setVoltage(volts);
    }
}
