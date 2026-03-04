package robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lib.RobotMode;
import lib.Tracer;
import lib.Tunable;
import lib.commands.CmdLogger;
import lib.hardware.CanBusLogger;
import lib.hardware.SignalRefresh;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import robot.constants.RobotConfig;
import robot.controllers.DriverController;
import robot.controllers.ManualOverrideController;
import robot.subsystems.Superstructure;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.drive.TunerConstants;
import robot.subsystems.intake.GroundIntake;
import robot.subsystems.serializer.Serializer;
import robot.subsystems.shooter.Shooter;

import java.util.Optional;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kRightRumble;

@SuppressWarnings("FieldCanBeLocal")
public class Robot extends LoggedRobot {
    static { // This is run before subsystems are created
        RobotConfig.initLoggingForMainBot();
    }

    private final Tunable<Double> pivotDebugVolts = Tunable.of("GroundIntake/Pivot/DemoVolts", 0);
    private final CanBusLogger canBusLogger = new CanBusLogger(TunerConstants.kCANBus);

    private final SwerveSubsystem drive = new SwerveSubsystem(RobotConfig.swerveCfg);
    private final Optional<IntakeSimulation> intakeSim = RobotConfig.createIntakeSim(drive);
    private final GroundIntake groundIntake = new GroundIntake(intakeSim);
    private final Serializer serializer = new Serializer(intakeSim);
    private final Shooter shooter = new Shooter();

    private final DriverController controller = new DriverController(0, RobotConfig.swerveCfg);
    private final ManualOverrideController manualController = new ManualOverrideController(1);

    private final Superstructure superstructure =
        new Superstructure(controller, drive, groundIntake, shooter, serializer);
    private final Autos autos = new Autos(drive, groundIntake, superstructure);

    public Robot() {
        setUseTiming(RobotMode.get() != RobotMode.REPLAY); // Run at max speed during replay mode
        Tunable.setEnabled(true);
        Tunable.of("DemoPose", Pose2d.kZero).onChange(drive::resetPose);
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

        controller.triangle().whileTrue(
            shooter.setFlywheelVelCmd(() -> RadiansPerSecond.of(30))
        );

        HubShiftUtil.initialize();
        for (int i = 1; i <= 5; i++) {
            double time = i;
            var shiftAboutToEnd =
                new Trigger(() -> (HubShiftUtil.getShiftedShiftInfo().remainingTime() < time));
            shiftAboutToEnd
                .and(RobotModeTriggers.teleop())
                .onTrue(controller.rumbleCmd(kRightRumble, 1.0).withTimeout(0.25));
        }

        RobotModeTriggers.test()
            .whileTrue(superstructure.shootInHubCmd());
//        RobotModeTriggers.autonomous()
//            .and(RobotMode::isSim)
//            .onTrue(Commands.runOnce(() -> SimulatedArena.getInstance().resetFieldForAuto()));
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
        HubShiftUtil.logData();
        Tracer.endCycle();
        Threads.setCurrentThreadPriority(false, 0);
    }
}
