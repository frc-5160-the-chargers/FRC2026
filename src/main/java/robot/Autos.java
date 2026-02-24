package robot;

import choreo.auto.AutoFactory;
import robot.constants.LoggingConfig;
import robot.misc.SharedData;
import robot.subsystems.Superstructure;
import robot.subsystems.drive.SwerveSubsystem;

public class Autos {
    private final AutoFactory autoFactory;

    public Autos(SwerveSubsystem drive, Superstructure superstructure) {
        autoFactory = new AutoFactory(
            drive::getPose, drive::resetPose,
            drive::followChoreoTraj,
            true, drive, LoggingConfig::logTrajectory
        );
    }
}
