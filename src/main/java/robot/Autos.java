package robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import lib.AutoVisualizer;
import lib.commands.CmdSequence;
import robot.constants.ChoreoTraj;
import robot.constants.RobotConfig;
import robot.subsystems.Superstructure;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.intake.GroundIntake;
import robot.subsystems.shooter.Shooter;
import robot.subsystems.shooter.Shooter.Target;

public class Autos {
    public enum TwoSwipeVersion {
        ONE, TWO, THREE
    }

    private final AutoFactory autoFactory;
    private final SwerveSubsystem drive;
    private final Superstructure superstructure;
    private final GroundIntake intake;
    private final Shooter shooter;
    private final AutoVisualizer autoViz;

    public Autos(
        SwerveSubsystem drive,
        GroundIntake intake,
        Superstructure superstructure,
        Shooter shooter
    ) {
        autoFactory = new AutoFactory(
            drive::getPose, drive::resetPose,
            target -> drive.followChoreoTraj(target, superstructure.rotationOverride),
            true, drive, RobotConfig::logTrajectory
        );
        this.superstructure = superstructure;
        this.intake = intake;
        this.shooter = shooter;
        this.drive = drive;
        this.autoViz = new AutoVisualizer("SelectedAuto", 1.0);
        CommandScheduler.getInstance().schedule(autoFactory.warmupCmd());
    }

    private AutoRoutine newAutoRoutine(String name) {
        var routine = autoFactory.newRoutine(name);
        routine.active()
            .onTrue(shooter.setIdleBehaviorToSpinupCmd())
            .onFalse(shooter.setIdleBehaviorToCoastCmd());
        return routine;
    }

    private AutoTrajectory newAutoTraj(AutoRoutine routine, ChoreoTraj trajFile, boolean mirrorLeftToRight) {
        var trajectory = trajFile.asAutoTraj(routine);

        return mirrorLeftToRight ? trajectory.mirrorY() : trajectory;
    }

    /** An auto routine that grabs balls from the center and shoots them, repeating twice. */
    public Command twoSwipe(boolean leftSide, TwoSwipeVersion version) {
        var traj1File = switch (version) {
            case ONE -> ChoreoTraj.CenterLoopPart1;
            case TWO -> ChoreoTraj.CenterLoopPart1_V2;
            case THREE -> ChoreoTraj.CenterLoopPart1_V3;
        };
        var isV1 = version == TwoSwipeVersion.ONE;
        var routine = newAutoRoutine("TwoSwipeFar");
        var traj1 = newAutoTraj(routine, traj1File, leftSide);
        var traj2 = newAutoTraj(routine, ChoreoTraj.CenterLoopPart2, leftSide);
        autoViz.setAutoSequence(traj1, traj2);

        routine.active().onTrue(superstructure.autoStartCmd(traj1));
        traj1.active()
            .whileTrue(superstructure.hubShotSpinupCmd(traj1));
        traj1.atTime(0.3)
            .onTrue(superstructure.intakeInAutoCmd(2.4, true));
        traj1.done().onTrue(
            CmdSequence.of(
                superstructure.shootInAutoCmd(Target.HUB, 2).withTimeout(4),
                traj2.spawnCmd()
            )
        );
        traj2.active()
            .whileTrue(superstructure.hubShotSpinupCmd(traj2))
            .whileTrue(superstructure.intakeInAutoCmd(3.3, false));
        traj2.done().onTrue(superstructure.shootInAutoCmd(Target.HUB, isV1 ? 2.0 : 1.5));

        return routine.cmd();
    }

    public Command twoSwipeDerail(boolean leftSide) {
        var routine = newAutoRoutine("TwoSwipeRush");
        var rushToCenter = newAutoTraj(routine, ChoreoTraj.CenterRush, leftSide);
        var traj1 = newAutoTraj(routine, ChoreoTraj.CenterLoopPart1_PostCenterRush, leftSide);
        var traj2 = newAutoTraj(routine, ChoreoTraj.CenterLoopPart2, leftSide);
        var brakeReq = new SwerveDriveBrake();
        autoViz.setAutoSequence(traj1, traj2);

        routine.active().onTrue(superstructure.autoStartCmd(rushToCenter));
        rushToCenter.atTime(1.33)
            .onTrue(
                drive.driveCmd(() -> brakeReq)
                    .alongWith(intake.deployCmd(1.0, drive::getRobotSpeeds))
                    .withTimeout(1.5)
                    .andThen(traj1.spawnCmd())
            );
        traj1.active()
            .whileTrue(superstructure.hubShotSpinupCmd(traj1))
            .whileTrue(superstructure.intakeInAutoCmd(2.4, true));
        traj1.done().onTrue(
            CmdSequence.of(
                superstructure.shootInAutoCmd(Target.HUB, 2).withTimeout(4),
                traj2.spawnCmd()
            )
        );
        traj2.active()
            .whileTrue(superstructure.hubShotSpinupCmd(traj2))
            .whileTrue(superstructure.intakeInAutoCmd(3.3, false));
        traj2.done().onTrue(superstructure.shootInAutoCmd(Target.HUB, 2));

        return routine.cmd();
    }

    /**
     * An auto routine that grabs balls from the middle, then grabs them
     * from either the human player station or the batch of balls on the ground.
     */
    public Command oneSwipeGrab(boolean leftSide) {
        var routine = newAutoRoutine("TwoSwipeClose");
        var centerScoop = newAutoTraj(routine, ChoreoTraj.CenterLoopPart1, leftSide);
        var closeGrab = newAutoTraj(
            routine,
            leftSide ? ChoreoTraj.CloseFuelGrab : ChoreoTraj.SubstationGrab,
            false
        );
        var closeScore = newAutoTraj(
            routine,
            leftSide ? ChoreoTraj.CloseFuelScore : ChoreoTraj.SubstationScore,
            false
        );
        autoViz.setAutoSequence(centerScoop, closeGrab, closeScore);

        routine.active().onTrue(superstructure.autoStartCmd(centerScoop));
        centerScoop.active()
            .whileTrue(superstructure.hubShotSpinupCmd(centerScoop));
        centerScoop.atTime(0.5)
            .onTrue(superstructure.intakeInAutoCmd(2.4, true));
        centerScoop.done().onTrue(
            CmdSequence.of(
                superstructure.shootInAutoCmd(Target.HUB, 1.5).withTimeout(4),
                closeGrab.spawnCmd()
            )
        );

        double shootTime = leftSide ? 0.7 : 1.0;
        routine.anyActive(closeGrab, closeScore)
            .whileTrue(superstructure.hubShotSpinupCmd(closeScore, shootTime))
            // If on the right side, the robot is grabbing fuel from the substation,
            // so the intake rollers are run at very low speed. Otherwise, the robot
            // is grabbing the alliance side prestaged balls from the floor.
            .whileTrue(intake.deployCmd(leftSide ? 1.15 : 0.1, drive::getRobotSpeeds));
        closeGrab.doneDelayed(leftSide ? 0.3 : 1.2)
            .onTrue(closeScore.spawnCmd());
        closeScore.atTime(shootTime)
            .onTrue(superstructure.shootInAutoCmd(Target.HUB, 1.5));

        return routine.cmd();
    }


    public Command stealFuelOverBump(boolean leftSide) {
        var routine = newAutoRoutine("StealFuelOverBump");
        var shootPreload = newAutoTraj(routine, ChoreoTraj.DriveBackAndShoot, !leftSide);
        var midlineGrab = newAutoTraj(routine, ChoreoTraj.BumpMidlineGrab, !leftSide);
        var midlineScore = newAutoTraj(routine, ChoreoTraj.BumpMidlineScore, !leftSide);
        autoViz.setAutoSequence(shootPreload, midlineGrab, midlineScore);

        routine.active()
            .onTrue(shootPreload.resetOdometry().andThen(shootPreload.spawnCmd()));
        shootPreload.active()
            .whileTrue(superstructure.hubShotSpinupCmd(shootPreload));
        shootPreload.done()
            .onTrue(
                superstructure.shootInAutoCmd(Target.HUB, 1000)
                    .withTimeout(3.0)
                    .andThen(midlineGrab.spawnCmd())
            );

        midlineGrab.atTime(1.7)
            .onTrue(superstructure.intakeInAutoCmd(1.0, true));
        midlineGrab.doneDelayed(2.0)
            .onTrue(midlineScore.spawnCmd());

        midlineScore.atTime(1.8)
            .onTrue(intake.moveUpForBumpTravelCmd());
        midlineScore.active().whileTrue(superstructure.hubShotSpinupCmd(midlineScore));
        midlineScore.done().onTrue(superstructure.shootInAutoCmd(Target.HUB, 1.0));

        return routine.cmd();
    }

    public Command midlineAuto() {
        var routine = newAutoRoutine("MidlineAuto");
        var driveBack = ChoreoTraj.DriveBackAndShoot.asAutoTraj(routine);
        autoViz.setAutoSequence(driveBack);

        routine.active().onTrue(
            driveBack.resetOdometry().andThen(driveBack.spawnCmd())
        );
        driveBack.active().whileTrue(intake.deployCmd(1.0, drive::getRobotSpeeds));
        driveBack.done().onTrue(
            superstructure.shootInAutoCmd(Target.HUB, 100)
        );

        return routine.cmd();
    }

    public void periodic() {
        autoViz.periodic();
    }
}
