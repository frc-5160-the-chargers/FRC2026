package robot.subsystems.drive.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import lib.RobotMode;
import lib.hardware.MotorStats;
import lib.hardware.SignalRefresh;
import org.littletonrobotics.junction.Logger;

/** Manages debug logging and error handling for swerve. */
class SwerveDebugging {
    private static final boolean LOG_HARDWARE_STATS = true;
    private static final int OVERFLOW_LIMIT = 40;

    private final MotorStats[] driveStats = new MotorStats[4];
    private final MotorStats[] steerStats = new MotorStats[4];
    private final boolean[] encodersConnected = new boolean[4];
    private final BaseStatusSignal[] connectedSignals = new BaseStatusSignal[5];

    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;

    SwerveDebugging(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain) {
        this.drivetrain = drivetrain;
        for (int i = 0; i < 4; i++) {
            connectedSignals[i] = drivetrain.getModule(i).getEncoder().getVersion();
        }
        connectedSignals[4] = drivetrain.getPigeon2().getVersion();
        SignalRefresh.register(10, drivetrain.getPigeon2().getNetwork(), connectedSignals);
    }

    void logData() {
        if (RobotMode.get() == RobotMode.REPLAY || !LOG_HARDWARE_STATS) return;
        for (int i = 0; i < 4; i++) {
            driveStats[i] = MotorStats.from(drivetrain.getModule(i).getDriveMotor());
            steerStats[i] = MotorStats.from(drivetrain.getModule(i).getSteerMotor());
            encodersConnected[i] = connectedSignals[i].getStatus().isOK();
        }
        Logger.recordOutput("SwerveSubsystem/DriveMotorData", driveStats);
        Logger.recordOutput("SwerveSubsystem/SteerMotorData", steerStats);
        Logger.recordOutput("SwerveSubsystem/EncodersConnected", encodersConnected);
        Logger.recordOutput("SwerveSubsystem/GyroConnected", connectedSignals[4].getStatus().isOK());
    }

    boolean isOverflowing(int poseEstBufferSize) {
        boolean res = poseEstBufferSize > OVERFLOW_LIMIT;
        Logger.recordOutput("SwerveSubsystem/PoseEstBufferOverflow", res);
        return res;
    }
}
