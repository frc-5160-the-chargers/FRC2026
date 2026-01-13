package robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import org.photonvision.estimation.TargetModel;

import java.util.Map;
import java.util.Optional;

/** Data structures used for vision processing. */
public class DataTypes {
    /** Camera-Specific Constants for AprilTag detection. */
    public record AprilTagCamConsts(
        String name,
        Transform3d robotCenterToCamera,
        AprilTagFieldLayout fieldLayout,
        double stdDevFactor,
        Optional<CameraIntrinsics> intrinsics
    ) {}

    /** A pose estimate originating from the vision cameras. */
    public record CamPoseEstimate(Pose2d pose, double timestampSecs, Vector<N3> deviations) {}

    /** Camera-Specific Constants for object detection. */
    public record MLCamConsts(
        String name,
        Map<String, TargetModel> availableObjects,
        Transform3d robotCenterToCamera,
        Optional<CameraIntrinsics> intrinsics
    ) {}

    /** Calibration data for a camera. */
    public record CameraIntrinsics(
        int width,
        int height,
        double fx,
        double fy,
        double cx,
        double cy,
        Vector<N8> distortionMatrix
    ) {
        public Matrix<N3, N3> cameraMatrix() {
            return MatBuilder.fill(Nat.N3(), Nat.N3(), fx, 0, cx, 0, fy, cy, 0, 0, 1);
        }

        public double horizontalFOV() {
            return 2.0 * Math.atan2(width, 2.0 * fx);
        }

        public double verticalFOV() {
            return 2.0 * Math.atan2(height, 2.0 * fy);
        }

        public double diagonalFOV() {
            return 2.0 * Math.atan2(Math.hypot(width, height) / 2.0, fx);
        }
    }

    /** This is a utility class. */
    private DataTypes() {}
}
