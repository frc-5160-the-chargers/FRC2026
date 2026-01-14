package robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import lib.Tunable;
import lib.commands.CmdLogger;
import lib.hardware.CanBusLogger;
import lib.hardware.SignalRefresh;
import lib.RobotMode;
import lib.Tracer;
import robot.constants.LoggingConfig;
import robot.subsystems.drive.SwerveConfig;
import robot.subsystems.drive.SwerveSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.drive.TunerConstants;

@SuppressWarnings("FieldCanBeLocal")
public class Robot extends LoggedRobot {
    private final Tunable<Pose2d> demoPose = Tunable.of("DemoPose", Pose2d.kZero);
    private final SwerveConfig swerveCfg = new SwerveConfig(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft, TunerConstants.FrontRight,
        TunerConstants.BackLeft, TunerConstants.BackRight
    );
    private final SwerveSubsystem drive = new SwerveSubsystem(swerveCfg);
    private final DriverController controller = new DriverController(swerveCfg);
    private final CanBusLogger canBusLogger = new CanBusLogger(TunerConstants.kCANBus);

    public Robot() {
        setUseTiming(RobotMode.get() != RobotMode.REPLAY); // Run at max speed during replay mode
        LoggingConfig.initForMainRobot();
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

    @Override
    public void robotPeriodic() {
        // TODO Disable setCurrentThreadPriority() if loop times are consistently over 20 ms
        Threads.setCurrentThreadPriority(true, 99);
        Tracer.trace("Signal Refresh", SignalRefresh::refreshAll);
        Tracer.trace("Cmd Scheduler", CommandScheduler.getInstance()::run);
        Logger.recordOutput(
            "LoggedRobot/MemoryUsageMb",
            (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1e6
        );
        canBusLogger.periodic();
        CmdLogger.periodic(true);
        Tracer.endCycle();
        Threads.setCurrentThreadPriority(false, 10);
    }
}
