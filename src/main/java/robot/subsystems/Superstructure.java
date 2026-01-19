package robot.subsystems;

import lombok.RequiredArgsConstructor;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.intake.GroundIntake;
import robot.subsystems.serializer.Serializer;
import robot.subsystems.shooter.Shooter;

@SuppressWarnings("unused")
@RequiredArgsConstructor
public class Superstructure {
    private final SwerveSubsystem drive;
    private final GroundIntake groundIntake;
    private final Serializer serializer;
    private final Shooter shooter;
}
