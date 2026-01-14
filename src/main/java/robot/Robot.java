package robot;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import lib.Tunable;
import lib.commands.CmdLogger;
import lib.hardware.CanBusLogger;
import lib.hardware.SignalRefresh;
import lib.RobotMode;
import lib.Tracer;
import org.ironmaple.simulation.SimulatedArena;
import robot.subsystems.drive.SwerveConfig;
import robot.constants.BuildConstants;
import robot.subsystems.drive.SwerveSubsystem;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import robot.subsystems.drive.TunerConstants;

import java.io.File;

@SuppressWarnings({"FieldCanBeLocal", "DataFlowIssue"})
public class Robot extends LoggedRobot {
    private static class EvergreenArena extends SimulatedArena {
        protected EvergreenArena() {
            super(new FieldMap() {});
        }
        @Override public void placeGamePiecesOnField() {}
    }

    static {
        SimulatedArena.overrideInstance(new EvergreenArena());
    }

    private final Tunable<Pose2d> demoPose = Tunable.of("DemoPose", Pose2d.kZero);
    private final SwerveConfig swerveCfg = new SwerveConfig(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft, TunerConstants.FrontRight,
        TunerConstants.BackLeft, TunerConstants.BackRight
    );
    private final SwerveSubsystem drive = new SwerveSubsystem(swerveCfg);
    private final DriverController controller = new DriverController(swerveCfg);

//    private final AprilTagCam cam = new AprilTagCam(VisionConsts.FL_CONSTS, drive::getTruePose);
//    private final CameraIO.RawData coralCamData = new CameraIO.RawData();
//    private final CameraIO coralCam = RobotMode.isSim()
//        ? new SimCameraIOForObjects(VisionConsts.CORAL_CAM_CONSTS, drive::truePose)
//        : new CameraIO(VisionConsts.CORAL_CAM_CONSTS.name());

    private final CanBusLogger canBusLogger = new CanBusLogger(TunerConstants.kCANBus);

    public Robot() {
        initLogging();
        drive.setDefaultCommand(
            drive.driveCmd(controller::getSwerveRequest)
        );
        drive.resetPose(new Pose2d(5, 7, Rotation2d.kZero));
        Tunable.setEnabled(true);
        demoPose.onChange(drive::resetPose);
        RobotModeTriggers.autonomous().whileTrue(
            drive.alignCmd(true, () -> new Pose2d(5, 7, Rotation2d.kZero))
        );
    }

    private void initLogging() {
        if (System.getenv("test") != null) {
            return; // Unit test, don't need logging here
        }
        if (RobotMode.get() == RobotMode.REPLAY) {
            setUseTiming(false);
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

    private void visionPeriodic() {
//        coralCam.refreshData(coralCamData);
//        Logger.processInputs("Cameras/CoralCam", coralCamData);
//        coralCamData.bestTarget().ifPresent(data -> {
//            Logger.recordOutput("CoralCam/Tx", data.yaw);
//            Logger.recordOutput("CoralCam/Ty", data.pitch);
//        });
//        drive.getInputs().ifPresent(
//            data -> cam.addHeadingData(data.poseEstFrames, data.timeOffsetSecs)
//        );
//        for (var m: cam.update()) {
//            drive.addVisionMeasurement(m);
//        }
    }

    @Override
    public void robotPeriodic() {
        // TODO Disable setCurrentThreadPriority() if loop times are consistently over 20 ms
        Threads.setCurrentThreadPriority(true, 99);
        Tracer.trace("Signal Refresh", SignalRefresh::refreshAll);
        Tracer.trace("Cmd Scheduler", CommandScheduler.getInstance()::run);
        Tracer.trace("Vision", this::visionPeriodic);
        Logger.recordOutput(
            "LoggedRobot/MemoryUsageMb",
            (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1e6
        );
        canBusLogger.periodic();
        CmdLogger.periodic(true);
        if (RobotMode.isSim()) {
            Logger.recordOutput("Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
            Logger.recordOutput("Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        }
        Tracer.endCycle();
        Threads.setCurrentThreadPriority(false, 10);
    }
}
