package robot.subsystems.drive.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import lib.RobotMode;
import lib.hardware.SignalRefresh;

import java.util.HashMap;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

/** Manages error handling for swerve. */
class SwerveAlertManager {
    private static final int OVERFLOW_LIMIT = 35, OVERFLOW_APPLICABLE_TIME_SECS = 5;

    private int overflowCount = 0;
    private final Alert overflowAlert = new Alert("", kError);
    private final Timer overflowApplicableTimer = new Timer();
    private final HashMap<Alert, BaseStatusSignal> checkupSignals = new HashMap<>();

    SwerveAlertManager(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drive) {
        String[] names = {"Front Left", "Front Right", "Back Left", "Back Right"};
        for (int i = 0; i < 4; i++) {
            var module = drive.getModule(i);
            checkupSignals.put(
                new Alert("Disconnected Drive Motor of " + names[i] + " Module.", kError),
                module.getDriveMotor().getVersion()
            );
            checkupSignals.put(
                new Alert("Disconnected Steer Motor of " + names[i] + " Module", kError),
                module.getSteerMotor().getVersion()
            );
            checkupSignals.put(
                new Alert("Disconnected Encoder of " + names[i] + " Module", kError),
                module.getEncoder().getVersion()
            );
        }
        var signalArray = checkupSignals.values().toArray(new BaseStatusSignal[0]);
        // all devices are on the same CAN bus/network
        SignalRefresh.register(5, drive.getPigeon2().getNetwork(), signalArray);
    }

    /** Updates the connection alerts for swerve. */
    void update() {
        for (var alertSignalPair: checkupSignals.entrySet()) {
            alertSignalPair.getKey().set(
                !alertSignalPair.getValue().getStatus().isOK()
            );
        }
    }

    boolean isOverflowing(int poseEstBufferSize) {
        if (RobotMode.get() == RobotMode.REPLAY
            || overflowApplicableTimer.get() < OVERFLOW_APPLICABLE_TIME_SECS
            || poseEstBufferSize <= OVERFLOW_LIMIT) {
            return false;
        }
        overflowCount += (poseEstBufferSize - OVERFLOW_LIMIT);
        overflowAlert.set(true);
        overflowAlert.setText(
            "Odometry Frame Buffer has overflowed by " + overflowCount + " frames; " +
            "replay may be impacted."
        );
        return true;
    }
}
