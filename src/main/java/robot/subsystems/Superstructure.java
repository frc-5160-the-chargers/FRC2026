package robot.subsystems;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import lombok.RequiredArgsConstructor;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.intake.GroundIntake;
import robot.subsystems.serializer.Serializer;
import robot.subsystems.shooter.Shooter;

import java.util.List;

@SuppressWarnings("unused")
@RequiredArgsConstructor
public class Superstructure {
    private static final List<Rectangle2d> noShootZones = List.of(
        new Rectangle2d(
            new Translation2d(3.8, 8),
            new Translation2d(5.3, 0)
        )
    );


    private final SwerveSubsystem drive;
    private final GroundIntake groundIntake;
    private final Serializer serializer;
    private final Shooter shooter;
}
