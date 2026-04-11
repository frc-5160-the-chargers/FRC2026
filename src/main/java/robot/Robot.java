package robot;

import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lib.RobotMode;
import lib.Tunable;
import lib.commands.CmdLogger;
import lib.commands.CmdSequence;
import lib.commands.CommandChooser;
import lib.hardware.CanBusLogger;
import lib.hardware.SignalRefresh;
import org.ironmaple.simulation.IntakeSimulation;
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
import java.util.Optional;

@SuppressWarnings("FieldCanBeLocal")
public class Robot extends LoggedRobot {
    static { // This is run before subsystems are created
        RobotConfig.initLoggingForMainBot();
    }


    private final Rectangle2d FIELD_BOUNDARIES = new Rectangle2d(
        Translation2d.kZero,
        new Translation2d(16.5, 8.0)
    );
    private final CanBusLogger canivoreLogger = new CanBusLogger(RobotConfig.CANIVORE);

    private final SwerveSubsystem drive = new SwerveSubsystem(RobotConfig.swerveCfg);
    private final Optional<IntakeSimulation> intakeSim = RobotConfig.createIntakeSim(drive);
    private final GroundIntake groundIntake = new GroundIntake(intakeSim);
    private final Serializer serializer = new Serializer(intakeSim);
    private final Shooter shooter = new Shooter();
    private final List<AprilTagCam> cameras =
        List.of(
            new AprilTagCam(drive.getSim(), VisionConsts.FL_CONSTS),
            new AprilTagCam(drive.getSim(), VisionConsts.FR_CONSTS)
        );

    private final DriverController driverPS5 = new DriverController(0, RobotConfig.swerveCfg);
    private final OperatorController operatorXbox = new OperatorController(1);

    private final Superstructure superstructure =
        new Superstructure(
            operatorXbox::getFlywheelSpeedAdjustment,
            driverPS5::getDesiredSpeeds,
            drive, groundIntake, shooter, serializer
        );
    private final Autos autos = new Autos(drive, groundIntake, superstructure, shooter);

    private final CommandChooser
        testChooser = new CommandChooser("TestModeChoices"),
        autoChooser = new CommandChooser("AutoModeChoices");

    public Robot() {
        setUseTiming(RobotMode.get() != RobotMode.REPLAY); // Run at max speed during replay mode
        // TODO check Tunable.java, for loop time reasons we totally disabled tuning so setEnabled() does nothing!
        Tunable.setEnabled(false);
        Tunable.of("DemoPose", Pose2d.kZero).onChange(drive::resetPose);
        setCompButtonBindings();
        setDefaultCommands();
        mapAutoAndTestModes();
        drive.resetPose(new Pose2d(2.5, 4, Rotation2d.kZero));
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        for (var cam: cameras) {
            cam.setPipeline(VisionConsts.AprilTagPipeline.HOME);
        }
        RobotController.setBrownoutVoltage(6.0);
        if (RobotMode.isSim()) SimulatedArena.getInstance().placeGamePiecesOnField();
    }

    private void setCompButtonBindings() {
        operatorXbox.leftBumper()
            .and(operatorXbox.leftTrigger())
            .whileTrue(groundIntake.deployCmd(1.3, drive::getRobotSpeeds));
        operatorXbox.leftBumper()
            .and(operatorXbox.leftTrigger().negate())
            .whileTrue(groundIntake.deployCmd(1.0, drive::getRobotSpeeds));
        operatorXbox.rightBumper().whileTrue(serializer.runCmd());
        operatorXbox.povUp().whileTrue(groundIntake.lowAgitateCmd());
        operatorXbox.povDown().whileTrue(groundIntake.outtakeCmd());
        operatorXbox.x().onTrue(shooter.setIdleBehaviorToSpinupCmd());
        operatorXbox.y().whileTrue(groundIntake.passiveAgitateCmd());

        driverPS5.touchpad()
            .multiPress(2, 1.0)
            .onTrue(
                Commands.runOnce(() -> drive.resetHeading(Rotation2d.kZero))
                    .ignoringDisable(true)
                    .withName("Drive Reset Heading")
            );
        driverPS5.povUp().whileTrue(
            drive.driveCmd(driverPS5::getSwishingSwerveRequest)
        );
        var brakeReq = new SwerveDriveBrake();
        driverPS5.povDown().whileTrue(drive.driveCmd(() -> brakeReq));
        driverPS5.R2().whileTrue(superstructure.spinupAndAimCmd(Shooter.Target.GROUND));
        driverPS5.circle().whileTrue(superstructure.spinupAndAimCmd(Shooter.Target.HUB));
        driverPS5.square().whileTrue(superstructure.manualHubShotCmd());

        RobotModeTriggers.disabled()
            .onTrue(shooter.setIdleBehaviorToCoastCmd());

        new Trigger(superstructure::canSerialize)
            .debounce(0.2, Debouncer.DebounceType.kFalling)
            .whileTrue(operatorXbox.notifySerializerReadyCmd());

        new Trigger(() -> FIELD_BOUNDARIES.contains(drive.getPose().getTranslation()))
            .whileFalse(operatorXbox.notifyOutOfBoundsCmd());

        initDashboard();
        if (RobotMode.isSim()) {
            RobotModeTriggers.autonomous()
                .whileTrue(
                    CmdSequence.of(
                        Commands.waitSeconds(20),
                        Commands.runOnce(() -> DriverStationSim.setEnabled(false))
                    )
                        .withName("Simulated Auto Ender")
                );
        }
    }

    private void initDashboard() {
//        tUttUtil.initialize();
//        for (int i = 1; i <= 5; i++) {
//            double time = i;
//            // the driver controller rumble plugin we use (scripts/driverstation/ps5_controller_rumble.py)
//            // adds a bit of delay due to NetworkTables; so, we run the driver rumble commands a bit earlier.
//            RobotModeTriggers.teleop()
//                .and(() -> (HubShiftUtil.getShiftedShiftInfo().remainingTime() < time + 0.3))
//                .onTrue(driverPS5.notifyHubShiftCmd());
//        }
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(
            drive.driveCmd(
                () -> driverPS5.getSwerveRequest(
                    superstructure.rotationOverride,
                    superstructure.shootingAtHub
                )
            )
        );
        groundIntake.setDefaultCommand(
            groundIntake.manualCmd(
                operatorXbox::getManualPivotVolts,
                operatorXbox.a()
            )
        );
        serializer.setDefaultCommand(serializer.stopCmd());
        shooter.setDefaultCommand(shooter.coastCmd());
    }

    private void mapAutoAndTestModes() {
        RobotModeTriggers.test()
            .whileTrue(testChooser.selectedCommandScheduler());
        RobotModeTriggers.autonomous()
            .whileTrue(autoChooser.selectedCommandScheduler());

        autoChooser.addCmd(
            "Reset Heading, shooter pointing towards DS",
            () -> Commands.runOnce(() -> drive.resetHeading(Rotation2d.k180deg))
        );
        autoChooser.addCmd("Two Swipe, Right", () -> autos.twoSwipe(false, false));
        autoChooser.addCmd("Two Swipe, Left", () -> autos.twoSwipe(true, false));
        autoChooser.addCmd("(V2) Two Swipe, Right", () -> autos.twoSwipe(false, true));
        autoChooser.addCmd("(V2) Two Swipe, Left", () -> autos.twoSwipe(true, true));
        autoChooser.addCmd("One Swipe + Substation, Right", () -> autos.oneSwipeGrab(false));
        autoChooser.addCmd("One Swipe + Ground Fuel, Left", () -> autos.oneSwipeGrab(true));
        autoChooser.addCmd("Midline Bump", () -> autos.stealFuelOverBump(false));
        autoChooser.addCmd("Two Swipe Derail, Right", () -> autos.twoSwipeDerail(false));
        autoChooser.addCmd("Two Swipe Derail, Left", () -> autos.twoSwipeDerail(true));

        testChooser.addCmd("Ground Intake Sim Test", () -> groundIntake.deployCmd(1.0, drive::getFieldSpeeds));
        testChooser.addCmd("Characterize Wheel Radius", drive::characterizeWheelRadiusCmd);
        testChooser.addCmd("Test Hub Shot", () -> superstructure.shootInAutoCmd(Shooter.Target.HUB, 2));
        testChooser.addCmd("Test Ferry", () -> superstructure.shootInAutoCmd(Shooter.Target.GROUND, 2));
        testChooser.addCmd("Test serializer", serializer::runCmd);
        testChooser.addCmd("Test Initial Deploy", () -> groundIntake.initialDeployCmd(1.0, ChassisSpeeds::new));
    }

    @Override
    public void robotPeriodic() {
        SignalRefresh.refreshAll();
        CommandScheduler.getInstance().run();
        periodicLogging();
        for (var cam: cameras) {
            for (var update: cam.update()) {
                drive.addVisionMeasurement(update);
            }
        }
    }

    private void periodicLogging() {
        if (RobotMode.isSim()) {
            Logger.recordOutput("Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        }
        double memoryMb = (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1e6;
        Logger.recordOutput("LoggedRobot/MemoryUsageMb", memoryMb);
        canivoreLogger.periodic();
        CmdLogger.periodic(true);
//        HubShiftUtil.logData();
    }
}
