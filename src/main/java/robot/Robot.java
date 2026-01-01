package robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import lib.TunableValues;
import lib.commands.CmdLogger;
import lib.hardware.SignalBatchRefresher;
import lib.RobotMode;
import lib.Tracer;
import org.ironmaple.simulation.SimulatedArena;
import robot.subsystems.drive.SwerveConfig;
import robot.constants.BuildConstants;
import robot.subsystems.drive.SwerveDrive;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;
import robot.vision.AprilTagCam;
import robot.vision.VisionConsts;

import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;

@SuppressWarnings({"FieldCanBeLocal", "DataFlowIssue"})
public class Robot extends LoggedRobot {
    private final SwerveDrive drive = new SwerveDrive(SwerveConfig.DEFAULT);
    private final DriverController controller = new DriverController();

    private final AprilTagCam cam = new AprilTagCam(VisionConsts.FL_CONSTS, drive::truePose);
//    private final CameraIO.RawData coralCamData = new CameraIO.RawData();
//    private final CameraIO coralCam = RobotMode.isSim()
//        ? new SimCameraIOForObjects(VisionConsts.CORAL_CAM_CONSTS, drive::truePose)
//        : new CameraIO(VisionConsts.CORAL_CAM_CONSTS.name());

    public Robot() {
        initLogging();
        drive.setDefaultCommand(
            drive.driveCmd(controller::getSwerveRequest)
        );
        drive.resetPose(new Pose2d(5, 7, Rotation2d.kZero));
        if (RobotMode.isSim()) {
            SimulatedArena.getInstance().placeGamePiecesOnField();
        }
        TunableValues.setTuningMode(true);
        autonomous().whileTrue(
            drive.pathfindCmd(() -> new Pose2d(5, 7, Rotation2d.kZero))
        );
    }

    private void initLogging() {
        if (RobotMode.get() == RobotMode.REPLAY) {
            setUseTiming(false);
            // Checks for an opened log file in AdvantageScope.
            String path = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(path));
            Logger.addDataReceiver(
                new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_replay"))
            );
        } else {
            var ntPublisher = new NT4Publisher();
            Logger.addDataReceiver(data -> {
                if (DriverStation.isFMSAttached()) return;
                ntPublisher.putTable(data);
            });
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.registerURCL(URCL.startExternal());
            Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
            Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
            Logger.recordMetadata("Timestamp", BuildConstants.BUILD_DATE);
            Logger.recordMetadata("GitDirty", switch (BuildConstants.DIRTY) {
                case 0 -> "All changes commited";
                case 1 -> "There are uncommited changes; replay might be inaccurate";
                default -> "Unknown";
            });
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
        var frames = drive.getInputs().poseEstFrames;
        for (var measurement: cam.updateWithTrigSolve(frames)) {
            drive.addVisionMeasurement(measurement);
        }
    }

    @Override
    public void robotPeriodic() {
        // TODO Disable setCurrentThreadPriority() if loop times are consistently over 20 ms
        Threads.setCurrentThreadPriority(true, 99);
        Tracer.trace("Signal Refresh", SignalBatchRefresher::refreshAll);
        Tracer.trace("Cmd Logger", () -> CmdLogger.periodic(true));
        Tracer.trace("Cmd Scheduler", CommandScheduler.getInstance()::run);
        Tracer.trace("Vision", this::visionPeriodic);
        Logger.recordOutput(
            "LoggedRobot/MemoryUsageMb",
            (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1e6
        );
        if (RobotMode.isSim()) {
            Logger.recordOutput("Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
            Logger.recordOutput("Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        }
        Tracer.endCycle();
        Threads.setCurrentThreadPriority(false, 10);
    }
}
