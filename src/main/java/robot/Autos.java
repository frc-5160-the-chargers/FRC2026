package robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import lib.commands.CmdSequence;
import robot.constants.ChoreoTraj;
import robot.constants.RobotConfig;
import robot.subsystems.Superstructure;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.intake.GroundIntake;
import robot.subsystems.shooter.Shooter;
import robot.subsystems.shooter.Shooter.Target;

public class Autos {
    private final AutoFactory autoFactory;
    private final SwerveSubsystem drive;
    private final Superstructure superstructure;
    private final GroundIntake intake;
    private final Shooter shooter;

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
//        CommandScheduler.getInstance().schedule(
//            autoFactory.warmupCmd()
//        );
    }

    private AutoRoutine newAutoRoutine(String name) {
        var routine = autoFactory.newRoutine(name);
        routine.active()
            .onTrue(shooter.setIdleBehaviorToSpinupCmd())
            .onFalse(shooter.setIdleBehaviorToCoastCmd());
        return routine;
    }

    private AutoTrajectory newAutoTraj(AutoRoutine routine, ChoreoTraj trajFile, boolean mirrorVertically) {
        var name = (mirrorVertically ? "mirrored_" : "") + trajFile.name();
        try {
            return ChoreoTraj.ALL_TRAJECTORIES.get(name).asAutoTraj(routine);
        } catch (Exception e) {
            // return null-op trajectory if invalid
            return routine.trajectory("");
        }
    }

    /** An auto routine that grabs balls from the center and shoots them. */
    public Command oneSwipe(boolean leftSide) {
        var routine = newAutoRoutine("OneSwipe");
        var grab1 = newAutoTraj(routine, ChoreoTraj.CenterGrab, leftSide);
        var score1 = newAutoTraj(routine, ChoreoTraj.CenterScore, leftSide);

        routine.active()
            .onTrue(CmdSequence.of(grab1.resetOdometry(), grab1.spawnCmd()));
        grab1.active()
            .whileTrue(intake.deployCmd(1, drive::getRobotSpeeds));
        grab1.doneDelayed(0.5)
            .onTrue(score1.spawnCmd());
        score1.doneDelayed(0.2)
            .onTrue(superstructure.shootInAutoCmd(Target.HUB, 100.0));

        return routine.cmd();
    }

    /** An auto routine that grabs balls from the center and shoots them, repeating twice. */
    public Command twoSwipeFar(boolean leftSide) {
        var routine = newAutoRoutine("TwoSwipeFar");
        var traj1 = newAutoTraj(routine, ChoreoTraj.CenterLoopFar, leftSide);
        var traj2 = newAutoTraj(routine, ChoreoTraj.CenterLoopClose, leftSide);

        routine.active().onTrue(superstructure.autoStartCmd(traj1));
        traj1.active()
            .whileTrue(superstructure.hubShotSpinupCmd(traj1));
        traj1.atTime(0.5)
            .onTrue(superstructure.intakeInAutoCmd(2.4));
        traj1.done().onTrue(
            CmdSequence.of(
                superstructure.shootInAutoCmd(Target.HUB, 2).withTimeout(4),
                traj2.spawnCmd()
            )
        );
        traj2.active()
            .whileTrue(superstructure.hubShotSpinupCmd(traj2))
            .whileTrue(superstructure.intakeInAutoCmd(3.3));
        traj2.done().onTrue(superstructure.shootInAutoCmd(Target.HUB, 2));

        return routine.cmd();
    }

    /**
     * An auto routine that grabs balls from the middle, then grabs them
     * from either the human player station or the batch of balls on the ground.
     */
    public Command twoSwipeClose(boolean leftSide) {
        var routine = newAutoRoutine("TwoSwipeClose");
        var centerScoop = newAutoTraj(routine, ChoreoTraj.CenterLoopFar, leftSide);
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

        routine.active().onTrue(superstructure.autoStartCmd(centerScoop));
        centerScoop.active()
            .whileTrue(superstructure.hubShotSpinupCmd(centerScoop));
        centerScoop.atTime(0.5)
            .onTrue(superstructure.intakeInAutoCmd(2.4));
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
}
