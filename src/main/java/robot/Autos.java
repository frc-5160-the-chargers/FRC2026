package robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import robot.constants.ChoreoTraj;
import robot.constants.RobotConfig;
import robot.subsystems.Superstructure;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.intake.GroundIntake;

public class Autos {
    private final AutoFactory autoFactory;
    private final Superstructure superstructure;
    private final GroundIntake intake;

    public Autos(SwerveSubsystem drive, GroundIntake intake, Superstructure superstructure) {
        autoFactory = new AutoFactory(
            drive::getPose, drive::resetPose,
            target -> drive.followChoreoTraj(target, superstructure.getRotationOverride()),
            true, drive, RobotConfig::logTrajectory
        );
        this.superstructure = superstructure;
        this.intake = intake;
    }

    public Command nearSide() {
        var routine = autoFactory.newRoutine("NearSide");
        var goToLoadingStation = ChoreoTraj.NearSideAuto$0.asAutoTraj(routine);
        var score1 = ChoreoTraj.NearSideAuto$1.asAutoTraj(routine);
        var allianceSideIntake = ChoreoTraj.NearSideAuto$2.asAutoTraj(routine);
        var score2 = ChoreoTraj.NearSideAuto$3.asAutoTraj(routine);

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
//        score1.done().onTrue(
//            superstructure.shootInHubCmd()
//                .withTimeout(5.0)
//                .andThen(allianceSideIntake.spawnCmd())
//        );
        allianceSideIntake.done().onTrue(
            Commands.waitSeconds(0.5)
                .andThen(score2.spawnCmd())
        );
        score2.active().whileTrue(
            superstructure.spinupForHubShotCmd(ChoreoTraj.NearSideAuto$3.endPoseBlue())
        );
//        score2.done().onTrue(superstructure.shootInHubCmd());

        return routine.cmd();
    }
}
