package robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Data(constants and global mutable state) shared across different components of the robot.
 * Avoid accessing this class from a hardware/IO layer (as that behavior will not be replayed).
 */
public class SharedData {
    // Mutable State
    public static Pose2d visionSimPose = Pose2d.kZero;
    public static int numSimulatedRobots = 0;

    /** Returns true when the current alliance is red. */
    public static boolean redAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }
}
