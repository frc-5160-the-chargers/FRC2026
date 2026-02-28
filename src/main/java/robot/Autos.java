package robot;

import choreo.auto.AutoFactory;
import robot.constants.LoggingConfig;
import robot.subsystems.Superstructure;
import robot.subsystems.drive.SwerveSubsystem;

public class Autos {
    private final AutoFactory autoFactory;

    public Autos(SwerveSubsystem drive, Superstructure superstructure) {
        autoFactory = new AutoFactory(
            drive::getPose, drive::resetPose,
            target -> drive.followChoreoTraj(target, superstructure.getRotationOverride()),
            true, drive, LoggingConfig::logTrajectory
        );
    }
}
