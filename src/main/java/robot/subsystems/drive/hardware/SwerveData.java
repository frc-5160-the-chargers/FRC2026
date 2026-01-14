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
    public double timeOffsetSecs = 0.0; // conversion factor from FPGA to Phoenix 6 timestamp.
    public boolean bufferOverflow = false;

    /** Data used for estimating pose in replay mode. */
    public record OdometryFrame(
        Rotation2d heading,
        double timestampSecs, // uses Phoenix 6 native timestamp instead of FPGA.
        SwerveModulePosition frontL,
        SwerveModulePosition frontR,
        SwerveModulePosition backL,
        SwerveModulePosition backR
    ) {
        public SwerveModulePosition[] positions() {
            return new SwerveModulePosition[] {frontL, frontR, backL, backR};
        }
    }
}
