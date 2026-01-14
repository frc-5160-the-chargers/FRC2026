package robot.constants;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.wpilibj.DriverStation;
import lib.RobotMode;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.File;

@SuppressWarnings("DataFlowIssue")
public class LoggingConfig {
    /** Logging config for the main robot. */
    public static void initForMainRobot() {
        if (System.getenv("test") != null) {
            return; // Unit test, don't need logging here
        }
        if (RobotMode.get() == RobotMode.REPLAY) {
            // Checks for an opened log file in AdvantageScope.
            String path = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(path));
            Logger.addDataReceiver(
                new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_replay"))
            );
        } else {
            var ntLogger = new NT4Publisher();
            Logger.addDataReceiver(data -> {
                if (DriverStation.isFMSAttached()) return;
                ntLogger.putTable(data);
            });
            boolean logToUsbDrive = RobotMode.isSim() || new File("/U/logs/").exists();
            Logger.addDataReceiver(
                logToUsbDrive ? new WPILOGWriter() : new WPILOGWriter("/home/lvuser/logs")
            );
            // Disable REV and CTRE logging because we don't really use them
            SignalLogger.enableAutoLogging(false);
            StatusLogger.disableAutoLogging();
            Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
            Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
            Logger.recordMetadata("Timestamp", BuildConstants.BUILD_DATE);
            Logger.recordMetadata("GitDirty", switch (BuildConstants.DIRTY) {
                case 0 -> "All changes commited";
                case 1 -> "There are uncommited changes; replay might be inaccurate";
                default -> "Unknown";
            });
            Logger.recordMetadata("LoggedToUSBDrive", logToUsbDrive ? "Yes" : "No");
        }
        Logger.start();
    }
}
