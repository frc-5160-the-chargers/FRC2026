package robot.subsystems.drive.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import lib.RobotMode;
import lib.hardware.MotorStats;
import lib.hardware.SignalRefresh;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

/** Manages debug logging and error handling for swerve. */
class SwerveDebugging {
    private static final boolean LOG_HARDWARE_STATS = true;
    private static final int OVERFLOW_LIMIT = 35, OVERFLOW_APPLICABLE_TIME_SECS = 5;

    private int overflowCount = 0;
    private final Alert overflowAlert = new Alert("", kError);
    private final Timer overflowApplicableTimer = new Timer();
    private final MotorStats[] driveStats = new MotorStats[4];
    private final MotorStats[] steerStats = new MotorStats[4];
    private final boolean[] encodersConnected = new boolean[4];
    private final BaseStatusSignal[] encoderSignals = new BaseStatusSignal[4];
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;

    SwerveDebugging(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain) {
        this.drivetrain = drivetrain;
        for (int i = 0; i < 4; i++) {
            encoderSignals[i] = drivetrain.getModule(i).getEncoder().getVersion();
        }
        SignalRefresh.register(10, drivetrain.getPigeon2().getNetwork(), encoderSignals);
    }

    void logData() {
        if (RobotMode.get() == RobotMode.REPLAY || !LOG_HARDWARE_STATS) return;
        for (int i = 0; i < 4; i++) {
            driveStats[i] = MotorStats.from(drivetrain.getModule(i).getDriveMotor());
            steerStats[i] = MotorStats.from(drivetrain.getModule(i).getSteerMotor());
            encodersConnected[i] = encoderSignals[i].getStatus().isOK();
        }
        Logger.recordOutput("SwerveSubsystem/DriveMotorData", driveStats);
        Logger.recordOutput("SwerveSubsystem/SteerMotorData", steerStats);
        Logger.recordOutput("SwerveSubsystem/EncodersConnected", encodersConnected);
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
            "Odometry Frame Buffer has overflowed by " + overflowCount + " frames total; " +
            "replay may be impacted."
        );
        return true;
    }
}
