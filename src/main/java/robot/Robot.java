package robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lib.RobotMode;
import lib.Tracer;
import lib.Tunable;
import lib.commands.CmdLogger;
import lib.commands.LoggedAutoChooser;
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

    private final Tunable<AngularVelocity> flywheelDebugVel =
        Tunable.of("Shooter/Flywheels/DebugVel", RadiansPerSecond.of(50));;

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

    private final LoggedAutoChooser
        testChooser = new LoggedAutoChooser("TestModeChoices"),
        autoChooser = new LoggedAutoChooser("AutoModeChoices");

    public Robot() {
        setUseTiming(RobotMode.get() != RobotMode.REPLAY); // Run at max speed during replay mode
        Tunable.setEnabled(true);
        Tunable.of("DemoPose", Pose2d.kZero).onChange(drive::resetPose);
        setButtonBindings();
        setDefaultCommands();
        mapAutoAndTestModes();

        CameraServer.startAutomaticCapture();
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    private void setButtonBindings() {
        controller.touchpad().multiPress(2, 0.3)
            .onTrue(Commands.runOnce(() -> drive.resetHeading(Rotation2d.kZero)).ignoringDisable(true).withName("Drive Reset Heading"));
        controller.triangle().whileTrue(
            serializer.runWhileTrueCmd(() -> true)
        );
        controller.circle().whileTrue(groundIntake.stowCmd());
        controller.square().whileTrue(groundIntake.intakeCmd());
        controller.cross().whileTrue(
            shooter.setFlywheelVelCmd(flywheelDebugVel::get)
        );
        controller.triangle().whileTrue(
            serializer.runWhileTrueCmd(() -> true)
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

    private void mapAutoAndTestModes() {
        RobotModeTriggers.test()
            .whileTrue(testChooser.autoScheduler());
        RobotModeTriggers.autonomous()
            .whileTrue(autoChooser.autoScheduler());

        autoChooser.addCmd("Near Side Auto", autos::nearSide);

        var hoodDebugDeg = Tunable.of("Shooter/Hood/DebugDeg", 50);
        testChooser.addCmd("Test Hub Shot", superstructure::shootInHubCmd);
        testChooser.addCmd("Test Ferry", superstructure::ferryCmd);
        testChooser.addCmd(
            "Set Hood Angle",
            () -> shooter.setHoodAngleCmd(() -> Rotation2d.fromDegrees(hoodDebugDeg.get()))
        );
        testChooser.addCmd(
            "Set Flywheel Vel",
            () -> shooter.setFlywheelVelCmd(flywheelDebugVel::get)
        );
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
