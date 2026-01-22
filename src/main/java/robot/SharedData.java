package robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Data(constants and global mutable state) shared across different components of the robot.
 * In general, avoid adding things to this class unless if it's an absolute necessity.
 */
public class SharedData {
    /** In sim, represents the true pose of the robot without any odometry drift. */
    public static Pose2d truePoseInSim = Pose2d.kZero;

    /** Returns true when the current alliance is red. */
    public static boolean redAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }
}
