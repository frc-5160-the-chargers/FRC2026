package robot.subsystems;

import lombok.RequiredArgsConstructor;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.pivot.hood.Hood;
import robot.subsystems.pivot.intake.IntakePivot;
import robot.subsystems.rollers.groundintake.GroundIntake;
import robot.subsystems.rollers.serializer.Serializer;
import robot.subsystems.shooter.Shooter;

@SuppressWarnings("unused")
@RequiredArgsConstructor
public class Superstructure {
    private final SwerveSubsystem drive;
    private final IntakePivot intakePivot;
    private final Hood hood;
    private final GroundIntake groundIntake;
    private final Serializer serializer;
    private final Shooter shooter;
}
