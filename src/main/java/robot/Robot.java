package robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
import robot.misc.DriverController;
import robot.misc.ManualOverrideController;
import robot.misc.SharedData;
import robot.subsystems.drive.SwerveConfig;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.drive.TunerConstants;
import robot.subsystems.intake.GroundIntake;

@SuppressWarnings("FieldCanBeLocal")
public class Robot extends LoggedRobot {
    static { // This is run before subsystems are created
        LoggingConfig.initForMainRobot();
    }

    private final Tunable<Pose2d> demoPose = Tunable.of("DemoPose", Pose2d.kZero);
    private final Tunable<Double> pivotDebugVolts = Tunable.of("GroundIntake/Pivot/DemoVolts", 0);
    private final CanBusLogger canBusLogger = new CanBusLogger(TunerConstants.kCANBus);
    private final SwerveConfig swerveCfg = new SwerveConfig(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft, TunerConstants.FrontRight,
        TunerConstants.BackLeft, TunerConstants.BackRight
    );

    private final SwerveSubsystem drive = new SwerveSubsystem(swerveCfg);
    private final GroundIntake groundIntake = new GroundIntake();

    private final DriverController controller = new DriverController(0, swerveCfg);
    private final ManualOverrideController manualController = new ManualOverrideController(1);

    public Robot() {
        setUseTiming(RobotMode.get() != RobotMode.REPLAY); // Run at max speed during replay mode
        demoPose.onChange(drive::resetPose);
        drive.setDefaultCommand(
            drive.driveCmd(() -> controller.getSwerveRequest(SharedData.rotOverride))
        );
        controller.touchpad().multiPress(2, 0.3)
            .onTrue(Commands.runOnce(() -> drive.resetHeading(Rotation2d.kZero)));
        controller.triangle().whileTrue(
            groundIntake
        )
        Tunable.setEnabled(true);

        groundIntake.setDefaultCommand(groundIntake.manualPivotCmd(true, manualController::getManualPivotVolts));
        manualController.x()
            .whileTrue(Commands.waitSeconds(2).andThen(groundIntake.intakeCmd()));
        manualController.y()
            .whileTrue(groundIntake.manualPivotCmd(false, pivotDebugVolts::get));
        if (RobotMode.isSim()) {
            RobotModeTriggers.test().onTrue(groundIntake.intakeCmd());
        }
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
