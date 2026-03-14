package robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lib.RobotMode;
import lib.Tunable;
import lib.commands.CmdLogger;
import lib.commands.LoggedAutoChooser;
import lib.hardware.CanBusLogger;
import lib.hardware.SignalRefresh;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import robot.constants.RobotConfig;
import robot.controllers.DriverController;
import robot.controllers.OperatorController;
import robot.subsystems.Superstructure;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.intake.GroundIntake;
import robot.subsystems.serializer.Serializer;
import robot.subsystems.shooter.Shooter;
import robot.vision.AprilTagCam;
import robot.vision.VisionConsts;

import java.util.List;

import static edu.wpi.first.units.Units.RadiansPerSecond;

@SuppressWarnings("FieldCanBeLocal")
public class Robot extends LoggedRobot {
    static { // This is run before subsystems are created
        RobotConfig.initLoggingForMainBot();
    }

    private final CanBusLogger canivoreLogger = new CanBusLogger(RobotConfig.CANIVORE);

    private final SwerveSubsystem drive = new SwerveSubsystem(RobotConfig.swerveCfg);
    private final GroundIntake groundIntake = new GroundIntake(RobotConfig.createIntakeSim(drive));
    private final Serializer serializer = new Serializer();
    private final Shooter shooter = new Shooter();
    private final List<AprilTagCam> cameras =
        List.of(
            new AprilTagCam(drive.getSim(), VisionConsts.FL_CONSTS),
            new AprilTagCam(drive.getSim(), VisionConsts.FR_CONSTS)
        );

    private final DriverController driver = new DriverController(0, RobotConfig.swerveCfg);
    private final OperatorController operator = new OperatorController(1);

    private final Superstructure superstructure =
        new Superstructure(
            operator::getFlywheelSpeedAdjustment,
            drive, groundIntake, shooter, serializer
        );
    private final Autos autos = new Autos(drive, groundIntake, superstructure, shooter);

    private final LoggedAutoChooser
        testChooser = new LoggedAutoChooser("TestModeChoices"),
        autoChooser = new LoggedAutoChooser("AutoModeChoices");

    public Robot() {
        setUseTiming(RobotMode.get() != RobotMode.REPLAY); // Run at max speed during replay mode
        Tunable.setEnabled(false);
        Tunable.of("DemoPose", Pose2d.kZero).onChange(drive::resetPose);
        setCompButtonBindings();
        setDefaultCommands();
        mapAutoAndTestModes();
        drive.resetPose(new Pose2d(2.5, 4, Rotation2d.kZero));

        CameraServer.startAutomaticCapture();
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    private void setCompButtonBindings() {
        operator.leftBumper().whileTrue(groundIntake.deployCmd(drive::getRobotSpeeds));
        operator.rightBumper().whileTrue(serializer.runCmd());
        operator.povUp().whileTrue(groundIntake.agitateCmd());
        operator.povDown().whileTrue(groundIntake.stowCmd());

        driver.touchpad()
            .multiPress(2, 1.0)
            .onTrue(
                Commands.runOnce(() -> drive.resetHeading(Rotation2d.kZero))
                    .ignoringDisable(true)
                    .withName("Drive Reset Heading")
            );
        driver.R2().whileTrue(superstructure.spinupAndAimCmd(Shooter.Target.GROUND));
        driver.circle().whileTrue(superstructure.spinupAndAimCmd(Shooter.Target.HUB));
        driver.square().whileTrue(superstructure.manualHubShotCmd());
        driver.cross().onTrue(shooter.setIdleBehaviorToSpinupCmd());

        RobotModeTriggers.disabled()
            .or(driver.triangle())
            .onTrue(shooter.setIdleBehaviorToCoastCmd());

        new Trigger(superstructure::canSerialize)
            .debounce(0.2, Debouncer.DebounceType.kFalling)
            .whileTrue(driver.notifySerializerReadyCmd())
            .whileTrue(operator.notifySerializerReadyCmd());

        initDashboard();
    }

    private void initDashboard() {
        HubShiftUtil.initialize();
        for (int i = 1; i <= 5; i++) {
            double time = i;
            // the driver controller rumble plugin we use (scripts/driverstation/ps5_controller_rumble.py)
            // adds a bit of delay due to NetworkTables; so, we run the driver rumble commands a bit earlier.
            RobotModeTriggers.teleop()
                .and(() -> (HubShiftUtil.getShiftedShiftInfo().remainingTime() < time + 0.3))
                .onTrue(driver.notifyHubShiftCmd());

            RobotModeTriggers.teleop()
                .and(() -> (HubShiftUtil.getShiftedShiftInfo().remainingTime() < time))
                .onTrue(operator.notifyHubShiftCmd());
        }
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(
            drive.driveCmd(
                () -> driver.getSwerveRequest(
                    superstructure.getRotationOverride(),
                    superstructure.getShotTarget()
                )
            )
        );
        groundIntake.setDefaultCommand(
            groundIntake.manualCmd(
                operator::getManualPivotVolts,
                operator.a()
            )
        );
        serializer.setDefaultCommand(serializer.stopCmd());
        shooter.setDefaultCommand(shooter.coastCmd());
    }

    private void mapAutoAndTestModes() {
        RobotModeTriggers.test()
            .whileTrue(testChooser.autoScheduler());
        RobotModeTriggers.autonomous()
            .whileTrue(autoChooser.autoScheduler());

        autoChooser.addCmd(
            "Reset Heading, shooter pointing towards DS",
            () -> Commands.runOnce(() -> drive.resetHeading(Rotation2d.k180deg))
        );
        autoChooser.addCmd("One Swipe, Right", () -> autos.oneSwipe(false));
        autoChooser.addCmd("One Swipe, Left", () -> autos.oneSwipe(true));
        autoChooser.addCmd("Two Swipe, Right", () -> autos.twoSwipe(false));
        autoChooser.addCmd("Two Swipe, Left", () -> autos.twoSwipe(true));

        testChooser.addCmd("Test Hub Shot", () -> superstructure.shootInAutoCmd(Shooter.Target.HUB));
        testChooser.addCmd("Test Ferry", () -> superstructure.shootInAutoCmd(Shooter.Target.GROUND));
        testChooser.addCmd(
            "Change heading to photonvision cam heading",
            () -> Commands.runOnce(() -> {
                var estimate = cameras.get(0).update().get(0);
                drive.resetHeading(estimate.pose().getRotation());
            })
        );
        testChooser.addCmd("Test serializer", serializer::runCmd);
        testChooser.addCmd(
            "Test Pulsing",
            () -> Commands.parallel(
                serializer.pulseCmd(),
                shooter.setVelocityCmd(() -> RadiansPerSecond.of(20))
            )
        );
    }

    @Override
    public void robotPeriodic() {
        // TODO Disable setCurrentThreadPriority() if loop times are consistently over 20 ms
        Threads.setCurrentThreadPriority(true, 1);
        SignalRefresh.refreshAll();
        CommandScheduler.getInstance().run();
        Logger.recordOutput(
            "LoggedRobot/MemoryUsageMb",
            (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1e6
        );
        if (RobotMode.isSim()) {
            Logger.recordOutput("Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        }
        canivoreLogger.periodic();
        for (var cam: cameras) {
            for (var update: cam.update()) {
                drive.addVisionMeasurement(update);
            }
        }
        CmdLogger.periodic(true);
        HubShiftUtil.logData();
        Threads.setCurrentThreadPriority(false, 0);
    }
}
