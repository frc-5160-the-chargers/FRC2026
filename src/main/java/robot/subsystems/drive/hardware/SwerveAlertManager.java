package robot.subsystems.drive.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj.Alert;
import lib.hardware.SignalBatchRefresher;

import java.util.ArrayList;
import java.util.HashMap;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

/** Manages pertinent alerts for a swerve drivetrain. */
class SwerveAlertManager {
    private final HashMap<Alert, BaseStatusSignal> checkupSignals = new HashMap<>();

    SwerveAlertManager(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> impl) {
        String[] names = {"Front Left", "Front Right", "Back Left", "Back Right"};
        for (int i = 0; i < 4; i++) {
            var module = impl.getModule(i);
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
        BaseStatusSignal.setUpdateFrequencyForAll(
            5.0, new ArrayList<>(checkupSignals.values())
        );
        SignalBatchRefresher.register(
            impl.getPigeon2().getNetwork(), checkupSignals.values()
        );
    }

    /** Updates the connection alerts for swerve. */
    void update() {
        for (var alertSignalPair: checkupSignals.entrySet()) {
            alertSignalPair.getKey().set(
                !alertSignalPair.getValue().getStatus().isOK()
            );
        }
    }
}
