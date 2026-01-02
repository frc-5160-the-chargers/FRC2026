package lib.hardware;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import lib.RobotMode;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

/** Logs information related to a CANivore on the robot. */
public class CanBusLogger {
    private final Alert
        canivoreAlert = new Alert("CANivore errors detected.", kError),
        mainBusAlert = new Alert("Main CANBus errors detected.", kError);
    private final String key;
    @Nullable private CANBus.CANBusStatus status = null;

    @SuppressWarnings({"BusyWait", "CallToPrintStackTrace"})
    public CanBusLogger(CANBus canBus) {
        key = canBus.getName();
        // Match RIO CAN sampling
        var thread = new Thread(() -> {
            while (true) {
                var statusTemp = canBus.getStatus();
                synchronized (this) {
                    status = statusTemp;
                }
                try {
                    Thread.sleep(400); // Match RIO CAN sampling
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        thread.setName("CanivoreReader - " + canBus.getName());
        thread.start();
    }

    /** Should be run in robotPeriodic(). */
    public void periodic() {
        if (RobotMode.get() == RobotMode.REPLAY) return;
        logCanivoreData();
        logMainBusData();
    }

    private void logCanivoreData() {
        CANBus.CANBusStatus current;
        synchronized (this) {
            if (status == null) return;
            current = status;
        }
        canivoreAlert.set(!current.Status.isOK());
        var stats = new CanivoreStats(
            current.BusUtilization, current.BusOffCount,
            current.TxFullCount, current.REC, current.TEC
        );
        Logger.recordOutput(key + "/Statistics", stats);
        Logger.recordOutput(key + "/Status", current.Status);
    }

    private void logMainBusData() {
        var status = RobotController.getCANStatus();
        mainBusAlert.set(status.transmitErrorCount > 0 || status.receiveErrorCount > 0);
    }

    private record CanivoreStats(
        float busUtilization, int offCount, int txFullCount,
        int receiveErrorCount, int transmitErrorCount
    ) {}
}
