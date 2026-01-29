package robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
    private final DriverController controller = new DriverController(0, swerveCfg);
    private final CanBusLogger canBusLogger = new CanBusLogger(TunerConstants.kCANBus);

    private final CommandXboxController xbox = new CommandXboxController(1);

    record Test(
        double hi,
        Testing testEnum
    ) {}

    enum Testing {
        A, B, C
    }

    public Robot() {
        setUseTiming(RobotMode.get() != RobotMode.REPLAY); // Run at max speed during replay mode
        demoPose.onChange(drive::resetPose);
        drive.setDefaultCommand(drive.driveCmd(controller::getSwerveRequest));
        climber.setDefaultCommand(climber.stop());
        controller.touchpad().multiPress(2, 0.3)
            .onTrue(Commands.runOnce(() -> drive.resetHeading(Rotation2d.kZero)));

        xbox.a().whileTrue(climber.setVoltage(6.0));
        xbox.b().whileTrue(climber.setPos(3));
        RobotModeTriggers.test()
            .whileTrue(drive.driveCmd(() -> controller.getFacingHubSwerveRequest(drive.getPose())));

        Tunable.setEnabled(true);
        Logger.recordOutput("Test", new Test(0, Testing.A));
        new Translation2d(5, 7);
        double x = 5 + 5;
    }



    @Override
    public void robotPeriodic() {
        // TODO Disable setCurrentThreadPriority() if loop times are consistently over 20 ms
        Threads.setCurrentThreadPriority(true, 1);
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
        Threads.setCurrentThreadPriority(false, 0);
    }
}
