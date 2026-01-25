package robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lib.RobotMode;
import lib.Tracer;
import lib.Tunable;
import lib.commands.CmdLogger;
import lib.hardware.CanBusLogger;
import lib.hardware.SignalRefresh;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import robot.constants.LoggingConfig;
import robot.controllers.DriverController;
import robot.subsystems.climber.Climber;
import robot.subsystems.drive.SwerveConfig;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.drive.TunerConstants;

@SuppressWarnings("FieldCanBeLocal")
public class Robot extends LoggedRobot {
    static { // This is run before subsystems are created
        LoggingConfig.initForMainRobot();
    }

    private final Tunable<Pose2d> demoPose = Tunable.of("DemoPose", Pose2d.kZero);
    private final SwerveConfig swerveCfg = new SwerveConfig(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft, TunerConstants.FrontRight,
        TunerConstants.BackLeft, TunerConstants.BackRight
    );
    private final SwerveSubsystem drive = new SwerveSubsystem(swerveCfg);
    private final Climber climber = new Climber();
    private final DriverController controller = new DriverController(swerveCfg);
    private final CanBusLogger canBusLogger = new CanBusLogger(TunerConstants.kCANBus);

    private final CommandXboxController xbox = new CommandXboxController(1);

    public Robot() {
        setUseTiming(RobotMode.get() != RobotMode.REPLAY); // Run at max speed during replay mode
        demoPose.onChange(drive::resetPose);
        drive.setDefaultCommand(drive.driveCmd(controller::getSwerveRequest));
        climber.setDefaultCommand(climber.stop());

        xbox.a().whileTrue(climber.setVoltage(6.0));                    
        xbox.b().whileTrue(climber.setPos(3));

        Tunable.setEnabled(true);
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
        if (RobotMode.isSim()) {
            Logger.recordOutput("Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        }
        canBusLogger.periodic();
        CmdLogger.periodic(true);
        Tracer.endCycle();
        Threads.setCurrentThreadPriority(false, 10);
    }
}
