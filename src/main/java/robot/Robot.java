package robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.units.measure.AngularVelocity;
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
import lib.commands.NonBlockingCmds;
import lib.hardware.CanBusLogger;
import lib.hardware.SignalRefresh;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.AutoLogOutput;
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
import robot.vision.AprilTagCam;
import robot.vision.VisionConsts;

import java.util.List;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kRightRumble;

@SuppressWarnings("FieldCanBeLocal")
public class Robot extends LoggedRobot {
    static { // This is run before subsystems are created
        RobotConfig.initLoggingForMainBot();
    }

    private final Tunable<AngularVelocity> flywheelDebugVel =
        Tunable.of("Shooter/Flywheels/DebugVel", RadiansPerSecond.of(50));

    private final CanBusLogger canBusLogger = new CanBusLogger(TunerConstants.kCANBus);

    private final SwerveSubsystem drive = new SwerveSubsystem(RobotConfig.swerveCfg);
    private final GroundIntake groundIntake = new GroundIntake(RobotConfig.createIntakeSim(drive));
    private final Serializer serializer = new Serializer();
    private final Shooter shooter = new Shooter();
    private final List<AprilTagCam> cameras =
        List.of(
            new AprilTagCam(drive.getSim(), VisionConsts.FL_CONSTS),
            new AprilTagCam(drive.getSim(), VisionConsts.FR_CONSTS)
        );

    private final DriverController controller = new DriverController(0, RobotConfig.swerveCfg);
    private final ManualOverrideController manualController = new ManualOverrideController(1);

    private final Superstructure superstructure =
        new Superstructure(
            manualController.rightBumper(),
            manualController::getFlywheelSpeedAdjustment,
            drive, groundIntake, shooter, serializer
        );
    private final Autos autos = new Autos(drive, groundIntake, superstructure, shooter);

    private final LoggedAutoChooser
        testChooser = new LoggedAutoChooser("TestModeChoices"),
        autoChooser = new LoggedAutoChooser("AutoModeChoices");

    @AutoLogOutput(key = "Experimental/Pulse Serializer")
    private boolean pulsingEnabled = false;

    @AutoLogOutput(key = "Experimental/Compensate for Robot Vel on Intake")
    private boolean closedLoopIntakeEnabled = true;

    public Robot() {
        setUseTiming(RobotMode.get() != RobotMode.REPLAY); // Run at max speed during replay mode
        Tunable.setEnabled(true);
        Tunable.of("DemoPose", Pose2d.kZero).onChange(drive::resetPose);
        setCompButtonBindings();
        setDefaultCommands();
        mapAutoAndTestModes();
        drive.resetPose(new Pose2d(2.5, 4, Rotation2d.kZero));

        CameraServer.startAutomaticCapture();
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    private void setCompButtonBindings() {
        controller.touchpad()
            .onTrue(Commands.runOnce(() -> drive.resetHeading(Rotation2d.kZero)).ignoringDisable(true).withName("Drive Reset Heading"));
        controller.L1()
            .or(manualController.leftBumper())
            .and(() -> !closedLoopIntakeEnabled)
            .whileTrue(groundIntake.intakeCmd());
        controller.L1()
            .or(manualController.leftBumper())
            .and(() -> closedLoopIntakeEnabled)
            .whileTrue(groundIntake.intakeCmd(drive::getRobotSpeeds));
        controller.R1()
            .whileTrue(groundIntake.stowCmd());

        controller.R2()
            .and(() -> !pulsingEnabled)
            .whileTrue(superstructure.shootCmd(Shooter.Target.GROUND));

        controller.circle()
            .and(() -> !pulsingEnabled)
            .whileTrue(superstructure.shootCmd(Shooter.Target.HUB));

        controller.R2()
            .and(() -> pulsingEnabled)
            .whileTrue(superstructure.shootCmd(Shooter.Target.GROUND, true, true));

        controller.circle()
            .and(() -> pulsingEnabled)
            .whileTrue(superstructure.shootCmd(Shooter.Target.HUB, true, true));

        controller.square().whileTrue(
            superstructure.visionlessHubShotCmd()
        );
        controller.cross()
            .onTrue(shooter.setIdleBehaviorToSpinupCmd());
        RobotModeTriggers.disabled()
            .or(controller.triangle())
            .onTrue(shooter.setIdleBehaviorToCoastCmd());

        initDashboard();

        manualController.povUp().onTrue(
            Commands.runOnce(() -> pulsingEnabled = true).withName("Enable Pulsing")
        );
        manualController.povDown().onTrue(
            Commands.runOnce(() -> closedLoopIntakeEnabled = true).withName("Enable Closed Loop Intake")
        );
        manualController.back().onTrue(
            Commands.runOnce(() -> {
                pulsingEnabled = false;
                closedLoopIntakeEnabled = false;
            })
                .withName("Disable Experimental")
        );
    }

    private void initDashboard() {
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
            drive.driveCmd(
                () -> {
                    Logger.recordOutput("Test", new Pose2d(drive.getPose().getTranslation(), superstructure.getRotationOverride().orElse(Rotation2d.kZero)));
                    return controller.getSwerveRequest(
                        superstructure.getRotationOverride(),
                        superstructure.getShotTarget()
                    );
                }
            )
        );
        groundIntake.setDefaultCommand(
            groundIntake.manualCmd(
                manualController::getManualPivotVolts,
                manualController.a()
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

        autoChooser.addCmd("(USE IF NO AUTO) Reset Heading to 180 deg", () -> Commands.runOnce(() -> drive.resetHeading(Rotation2d.k180deg)));
        autoChooser.addCmd("Near Side Auto", autos::nearSide);
        autoChooser.addCmd("Right Side Far Auto", autos::rightSide);
        autoChooser.addCmd("(Mirrored) Right Side Far Auto", autos::rightSideMirrored);

        testChooser.addCmd("Test Hub Shot", () -> superstructure.shootCmd(Shooter.Target.HUB));
        testChooser.addCmd("Test Ferry", () -> superstructure.shootCmd(Shooter.Target.GROUND));
        testChooser.addCmd(
            "Set Flywheel Vel",
            () -> shooter.setVelocityCmd(flywheelDebugVel::get)
        );
        testChooser.addCmd(
            "Change heading to photonvision cam heading",
            () -> Commands.runOnce(() -> {
                var estimate = cameras.get(0).update().get(0);
                drive.resetHeading(estimate.pose().getRotation());
            })
        );
        testChooser.addCmd("Just Aim", superstructure::hubAimCmd);
        testChooser.addCmd(
            "Test serializer",
            () -> serializer.runCmd(() -> true)
        );
        testChooser.addCmd(
            "Test Pulsing",
            () -> NonBlockingCmds.parallel(
                serializer.pulseCmd(() -> true),
                shooter.setVelocityCmd(() -> RadiansPerSecond.of(20))
            ).withName("Pulse Testing")
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
        canBusLogger.periodic();
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
