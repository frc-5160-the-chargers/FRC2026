package robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private void setupAutoRoutine(AutoRoutine routine) {
        routine.active()
            .onTrue(shooter.setIdleBehaviorToSpinupCmd())
            .onFalse(shooter.setIdleBehaviorToCoastCmd());
    }

    public Command nearSide() {
        var routine = autoFactory.newRoutine("NearSide");
        var goToLoadingStation = ChoreoTraj.NearSideAuto$0.asAutoTraj(routine);
        var score1 = ChoreoTraj.NearSideAuto$1.asAutoTraj(routine);
        var allianceSideIntake = ChoreoTraj.NearSideAuto$2.asAutoTraj(routine);
        var score2 = ChoreoTraj.NearSideAuto$3.asAutoTraj(routine);

        setupAutoRoutine(routine);
        routine.active().onTrue(
            goToLoadingStation.resetOdometry().andThen(goToLoadingStation.spawnCmd())
        );
        routine.anyActive(goToLoadingStation, allianceSideIntake)
            .whileTrue(intake.intakeCmd());

        goToLoadingStation.done().onTrue(
            Commands.waitSeconds(2).andThen(score1.spawnCmd())
        );
        score1.active().whileTrue(
            superstructure.spinupForHubShotCmd(ChoreoTraj.NearSideAuto$1.endPoseBlue())
        );
        score1.done().onTrue(
            superstructure.shootCmd(Shooter.Target.HUB)
                .withTimeout(5.0)
                .andThen(allianceSideIntake.spawnCmd())
        );
        allianceSideIntake.done().onTrue(
            Commands.waitSeconds(0.5)
                .andThen(score2.spawnCmd())
        );
        score2.active().whileTrue(
            superstructure.spinupForHubShotCmd(ChoreoTraj.NearSideAuto$3.endPoseBlue())
        );
        score2.done().onTrue(superstructure.shootCmd(Shooter.Target.HUB));

        return routine.cmd();
    }

    public Command rightSide() {
        var routine = autoFactory.newRoutine("RightSide");
        var grab1 = ChoreoTraj.FarSideAuto_Grab1.asAutoTraj(routine);
        var score1 = ChoreoTraj.FarSideAuto_Score1.asAutoTraj(routine);

        setupAutoRoutine(routine);
        routine.active().onTrue(grab1.resetOdometry().andThen(grab1.spawnCmd()));
        grab1.active().whileTrue(intake.intakeCmd(drive::getFieldSpeeds));
        grab1.done().onTrue(Commands.waitSeconds(1).andThen(score1.spawnCmd()));
        score1.doneDelayed(0.2)
            .onTrue(superstructure.shootCmd(Shooter.Target.HUB));

        return routine.cmd();
    }

    public Command rightSideMirrored() {
        var routine = autoFactory.newRoutine("RightSideMirrored");
        var grab1 = ChoreoTraj.mirrored_FarSideAuto_Grab1.asAutoTraj(routine);
        var score1 = ChoreoTraj.mirrored_FarSideAuto_Score1.asAutoTraj(routine);

        setupAutoRoutine(routine);
        routine.active().onTrue(grab1.resetOdometry().andThen(grab1.spawnCmd()));
        grab1.active().whileTrue(intake.intakeCmd(drive::getFieldSpeeds));
        grab1.done().onTrue(Commands.waitSeconds(1).andThen(score1.spawnCmd()));
        score1.doneDelayed(0.2)
            .onTrue(superstructure.shootCmd(Shooter.Target.HUB));

        return routine.cmd();
    }
}
