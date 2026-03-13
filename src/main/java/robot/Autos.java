package robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.commands.CmdSequence;
import robot.constants.ChoreoTraj;
import robot.constants.RobotConfig;
import robot.subsystems.Superstructure;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.intake.GroundIntake;
import robot.subsystems.shooter.Shooter;
import robot.subsystems.shooter.Shooter.IdleBehavior;

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
            target -> drive.followChoreoTraj(target, superstructure.getRotationOverride()),
            true, drive, RobotConfig::logTrajectory
        );
        this.superstructure = superstructure;
        this.intake = intake;
        this.shooter = shooter;
        this.drive = drive;
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
    public Command oneSwipe(boolean mirrorVertically) {
        var routine = newAutoRoutine("OneSwipe");
        var grab1 = newAutoTraj(routine, ChoreoTraj.OneSwipe_Grab1, mirrorVertically);
        var score1 = newAutoTraj(routine, ChoreoTraj.OneSwipe_Score1, mirrorVertically);

        routine.active()
            .onTrue(CmdSequence.of(grab1.resetOdometry(), grab1.spawnCmd()));
        grab1.active()
            .whileTrue(intake.intakeCmd(drive::getFieldSpeeds));
        grab1.doneDelayed(0.5)
            .onTrue(score1.spawnCmd());
        score1.doneDelayed(0.2)
            .onTrue(superstructure.shootCmd(Shooter.Target.HUB));

        return routine.cmd();
    }

    /** An auto routine that grabs balls from the center and shoots them, repeating twice. */
    public Command twoSwipe(boolean mirrorVertically) {
        var routine = newAutoRoutine("TwoSwipe");
        var traj1 = newAutoTraj(routine, ChoreoTraj.TwoSwipe_V1_1, mirrorVertically);
        var traj2 = newAutoTraj(routine, ChoreoTraj.TwoSwipe_V1_2, mirrorVertically);

        routine.active()
            .onTrue(CmdSequence.of(traj1.resetOdometry(), traj1.spawnCmd()));
        traj1.active()
            // note that this works even with a mirrored trajectory (which doesn't have the same end pose)
            // because the hub is perfectly centered in the vertical direction.
            .whileTrue(superstructure.spinupForHubShotCmd(ChoreoTraj.TwoSwipe_V1_1.endPoseBlue()))
            .whileTrue(
                intake.intakeCmd(drive::getFieldSpeeds).until(traj1.atTime(4.3))
            );
        traj1.done().onTrue(
            CmdSequence.of(
                superstructure.shootCmd(Shooter.Target.HUB).withTimeout(4.0),
                traj2.spawnCmd()
            )
        );
        traj2.active()
            .whileTrue(superstructure.spinupForHubShotCmd(ChoreoTraj.TwoSwipe_V1_2.endPoseBlue()))
            .whileTrue(
                intake.intakeCmd(drive::getFieldSpeeds).until(traj2.atTime(6.0))
            );
        traj2.done().onTrue(superstructure.shootCmd(Shooter.Target.HUB));

        return routine.cmd();
    }
}
