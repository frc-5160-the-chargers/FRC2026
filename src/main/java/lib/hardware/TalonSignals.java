package lib.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import edu.wpi.first.wpilibj.DriverStation;
import lib.Convert;
import lib.Retry;

import java.util.ArrayList;
import java.util.List;

/**
 * A utility class that reduces boilerplate around refreshing {@link MotorData}
 * for a group of TalonFX/TalonFXS motors moving the same mechanism.
 */
public class TalonSignals {
    public final BaseStatusSignal position, velocity, voltage;
    private final List<BaseStatusSignal>
        motorTemp = new ArrayList<>(),
        supplyCurrent = new ArrayList<>(),
        torqueCurrent = new ArrayList<>();

    /**
     * Creates a new TalonSignals object. To use this class,
     * <b>YOU MUST CALL SignalBatchRefresher.refreshAll() in robotPeriodic().</b>
     */
    public TalonSignals(CommonTalon leader, CommonTalon... followers) {
        position = leader.getPosition();
        velocity = leader.getVelocity();
        voltage = leader.getMotorVoltage();
        addExtSignals(leader);
        for (var follower: followers) {
            addExtSignals(follower);
        }

        var signalList = new ArrayList<>(List.of(position, velocity, voltage));
        signalList.addAll(motorTemp);
        signalList.addAll(supplyCurrent);
        signalList.addAll(torqueCurrent);
        Retry.ctreConfig(
            4, "Status signal frequency set failed",
            () -> BaseStatusSignal.setUpdateFrequencyForAll(50, signalList)
        );
        SignalBatchRefresher.register((ParentDevice) leader, signalList);
    }

    /**
     * Refreshes a {@link MotorData} object with data from the signals.
     * @param inputs the motor data to refresh.
     */
    public void refresh(MotorData inputs) {
        int numMotors = motorTemp.size();
        inputs.setNumMotors(numMotors);
        inputs.errorAsString = "";
        inputs.volts = voltage.getValueAsDouble();
        inputs.radians = position.getValueAsDouble() * Convert.ROTATIONS_TO_RADIANS;
        inputs.radiansPerSec = velocity.getValueAsDouble() * Convert.ROTATIONS_TO_RADIANS;
        for (int i = 0; i < numMotors; i++) {
            inputs.supplyAmps[i] = supplyCurrent.get(i).getValueAsDouble();
            inputs.tempCelsius[i] = motorTemp.get(i).getValueAsDouble();
            inputs.appliedAmps[i] = torqueCurrent.get(i).getValueAsDouble();
        }
        var tempStatus = motorTemp.get(0).getStatus();
        if (!tempStatus.isOK()) {
            inputs.errorAsString = tempStatus.toString();
        } else if (inputs.tempCelsius[0] == 0) {
            DriverStation.reportError("You aren't calling SignalBatchRefresher.refreshAll() in robotPeriodic().", false);
        }
    }

    private void addExtSignals(CommonTalon motor) {
        motorTemp.add(motor.getDeviceTemp());
        supplyCurrent.add(motor.getSupplyCurrent());
        torqueCurrent.add(motor.getTorqueCurrent());
    }
}
