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
import robot.controllers.DriverController;
import robot.controllers.ManualOverrideController;
import robot.subsystems.Superstructure;
import robot.subsystems.drive.SwerveConfig;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.drive.TunerConstants;
import robot.subsystems.intake.GroundIntake;
import robot.subsystems.serializer.Serializer;
import robot.subsystems.shooter.Shooter;

import static edu.wpi.first.units.Units.Inches;

@SuppressWarnings("FieldCanBeLocal")
public class Robot extends LoggedRobot {
    static { // This is run before subsystems are created
        LoggingConfig.initForMainRobot();
    }

    private final Tunable<Double> pivotDebugVolts = Tunable.of("GroundIntake/Pivot/DemoVolts", 0);
    private final CanBusLogger canBusLogger = new CanBusLogger(TunerConstants.kCANBus);
    private final SwerveConfig swerveCfg = new SwerveConfig(
        "Swerve",
        Inches.of(27 + 2 * 3.5),
        Inches.of(26 + 2 * 3.5),
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft, TunerConstants.FrontRight,
        TunerConstants.BackLeft, TunerConstants.BackRight
    );

    private final SwerveSubsystem drive = new SwerveSubsystem(swerveCfg);
    private final GroundIntake groundIntake = new GroundIntake(drive.getSim());
    private final Shooter shooter = new Shooter();
    private final Serializer serializer = new Serializer();

    private final DriverController controller = new DriverController(0, swerveCfg);
    private final ManualOverrideController manualController = new ManualOverrideController(1);

    private final Superstructure superstructure =
        new Superstructure(controller, drive, groundIntake, shooter, serializer);

    public Robot() {
        setUseTiming(RobotMode.get() != RobotMode.REPLAY); // Run at max speed during replay mode
        Tunable.setEnabled(true);
        Tunable.of("DemoPose", Pose2d.kZero).onChange(drive::resetPose);
//        serializer.setSimGamePieceRemover(groundIntake.sim::obtainGamePieceFromIntake);
//        serializer.setSimGamePiecesCounter(groundIntake.sim::getGamePiecesAmount);
        setButtonBindings();
        setDefaultCommands();
    }

    private void setButtonBindings() {
        controller.touchpad().multiPress(2, 0.3)
            .onTrue(Commands.runOnce(() -> drive.resetHeading(Rotation2d.kZero)));
        controller.triangle().whileTrue(
            groundIntake.manualPivotCmd(pivotDebugVolts::get)
        );
        controller.circle().whileTrue(groundIntake.stowCmd());
        controller.square().whileTrue(groundIntake.intakeCmd());
        if (RobotMode.isSim()) {
            RobotModeTriggers.test()
                .whileTrue(superstructure.shootInHubCmd());
            var demoAngle = Tunable.of("DemoHoodAngleDeg", 90);
            RobotModeTriggers.autonomous()
                .whileTrue(shooter.setHoodAngleCmd(() -> Rotation2d.fromDegrees(demoAngle.get())));
        }
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(
            drive.driveCmd(() -> controller.getSwerveRequest(superstructure.getRotationOverride()))
        );
        groundIntake.setDefaultCommand(
            groundIntake.manualPivotCmd(manualController::getManualPivotVolts)
        );
        shooter.setDefaultCommand(shooter.stopCmd());
        serializer.setDefaultCommand(serializer.stopCmd());
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
