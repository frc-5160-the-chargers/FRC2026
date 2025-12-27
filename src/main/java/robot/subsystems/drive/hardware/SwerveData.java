package robot.subsystems.drive.hardware;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

/**
 * Data received from CTRE's swerve drivetrain every 0.02 seconds.
 * Use {@link SwerveDataAutoLogged} in place of this class.
 */
@AutoLog
public class SwerveData {
    public OdometryFrame[] poseEstFrames = {};
    public SwerveModuleState[] currentStates = {};
    public SwerveModuleState[] desiredStates = {};
    public ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
    public Pose2d notReplayedPose = Pose2d.kZero;
    public boolean poseEstValid = true;

    /** Data used for estimating pose in replay mode. */
    public record OdometryFrame(
        Rotation2d heading,
        double timestampSecs,
        SwerveModulePosition tl,
        SwerveModulePosition tr,
        SwerveModulePosition bl,
        SwerveModulePosition br
    ) {
        public SwerveModulePosition[] positions() {
            return new SwerveModulePosition[] {tl, tr, bl, br};
        }
    }
}
