package robot.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import lib.RobotMode;
import lib.Tracer;
import robot.vision.Structs.CamPoseEstimate;
import robot.vision.Structs.AprilTagCamConsts;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
import static robot.vision.VisionConsts.*;

/** Handles processing pose estimates from an apriltag camera. */
public class AprilTagCam {
    private final AprilTagCamConsts consts;
    private final PhotonPoseEstimator poseEst;
    private final CameraIO io;
    private final CameraIO.RawData inputs = new CameraIO.RawData();

    private final List<Integer> fiducialIds = new ArrayList<>();
    private final List<Pose3d> poses = new ArrayList<>();

    public AprilTagCam(AprilTagCamConsts consts, Supplier<Pose2d> simPoseSupplier) {
        this.consts = consts;
        this.io = RobotMode.isSim()
            ? new SimCameraIOForTags(consts, simPoseSupplier)
            : new CameraIO(consts.name());
        this.poseEst = new PhotonPoseEstimator(
            consts.fieldLayout(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            consts.robotCenterToCamera()
        );
    }

    private boolean shouldLog() {
        return !DriverStation.isFMSAttached();
    }
    
    private String key(String path) {
        return "Cameras/" + consts.name() + "/" + path;
    }

    /**
     * Fetches the latest pose estimates from this camera,
     * using single-tag estimation if applicable.
     */
    public List<CamPoseEstimate> update(Rotation2d heading, double headingTimestampSecs) {
        poseEst.setMultiTagFallbackStrategy(
            DriverStation.isDisabled()
                ? PoseStrategy.LOWEST_AMBIGUITY
                : PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
        );
        poseEst.addHeadingData(headingTimestampSecs, heading);
        return update();
    }

    /** Fetches the latest pose estimates from this camera. */
    public List<CamPoseEstimate> update() {
        Tracer.startTrace("Vision Update (" + consts.name() + ")");

        Tracer.startTrace("Poll Data");
        io.refreshData(inputs);
        Logger.processInputs(key(""), inputs);
        Tracer.endTrace();

        var poseEstimates = new ArrayList<CamPoseEstimate>();
        if (RobotMode.get() == RobotMode.REAL && !inputs.connected) {
            return poseEstimates;
        }

        int ambHighCount = 0;
        int errHighCount = 0;
        fiducialIds.clear();
        poses.clear();
        for (var result: inputs.results) {
            // ignores result if ambiguity is exceeded or if there is no targets.
            if (result.targets.isEmpty()) {
                continue;
            }
            boolean ambiguityExceeded = true;
            // Computes the standard deviations of the pose,
            // scaling off distance from the target, z error, and # of targets.
            double tagDistSum = 0.0;
            double tagAreaSum = 0.0;
            for (var target: result.targets) {
                ambiguityExceeded = ambiguityExceeded && target.poseAmbiguity > MAX_AMBIGUITY;
                tagDistSum += target.bestCameraToTarget.getTranslation().getNorm();
                tagAreaSum += target.area;
                if (shouldLog()) fiducialIds.add(target.fiducialId);
            }
            if (ambiguityExceeded) {
                ambHighCount++;
                continue;
            }

            // updates the pose, and makes sure that the estimated pose
            // has a z coordinate near 0 and x and y coordinates within the field.
            var poseEstimate = poseEst.update(result);
            if (poseEstimate.isEmpty()) continue;
            var pose = poseEstimate.get().estimatedPose;
            var timestamp = poseEstimate.get().timestampSeconds;
            if (Math.abs(pose.getZ()) > MAX_Z_ERROR.in(Meters)
                || pose.getX() < 0.0
                || pose.getX() > FIELD_LAYOUT.getFieldLength()
                || pose.getY() < 0.0
                || pose.getY() > FIELD_LAYOUT.getFieldWidth()) {
                errHighCount++;
                continue;
            }

            // Calculates standard deviations
            double areaSumMultiplier = Math.pow(result.targets.size() / Math.abs(tagAreaSum), 0.2);
            double stdDevMultiplier = Math.pow(tagDistSum / result.targets.size(), 2) / result.targets.size();
            stdDevMultiplier *= Math.pow(Z_ERROR_SCALAR, Math.abs(pose.getZ()));
            stdDevMultiplier *= Math.max(areaSumMultiplier, 1);
            if (result.targets.size() <= 1) stdDevMultiplier *= SINGLE_TAG_SCALAR;
            double linearStdDev = stdDevMultiplier * LINEAR_STD_DEV_BASELINE * consts.stdDevFactor();

            if (shouldLog()) poses.add(pose);
            var stdDevs = VecBuilder.fill(linearStdDev, linearStdDev, ANGULAR_STD_DEV);
            poseEstimates.add(new CamPoseEstimate(pose.toPose2d(), timestamp, stdDevs));
        }
        Tracer.endTrace();

        // logs relevant data
        if (shouldLog()) {
            int[] ids = new int[fiducialIds.size()];
            for (int i = 0; i < fiducialIds.size(); i++) {
                ids[i] = fiducialIds.get(i);
            }
            Logger.recordOutput(key("fiducialIds"), ids);
            Logger.recordOutput(key("numAmbiguityExceeded"), ambHighCount);
            Logger.recordOutput(key("numErrExceeded"), errHighCount);
            Logger.recordOutput(key("poses"), poses.toArray(new Pose3d[0]));
        }

        return poseEstimates;
    }
}