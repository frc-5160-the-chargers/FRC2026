package robot.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import lib.RobotMode;
import lib.Tracer;
import robot.subsystems.drive.hardware.SwerveData.OdometryFrame;
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
    
    private String key(String path) {
        return "Cameras/" + consts.name() + "/" + path;
    }

    /**
     * Fetches the latest pose estimates from this camera,
     * using single-tag estimation if applicable.
     * @param odoFrames Fetched through drive.getInputs().poseEstFrames.
     */
    public List<CamPoseEstimate> updateWithTrigSolve(OdometryFrame[] odoFrames) {
        poseEst.setMultiTagFallbackStrategy(
            DriverStation.isDisabled() || odoFrames.length == 0
                ? PoseStrategy.LOWEST_AMBIGUITY
                : PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
        );
        for (var frame: odoFrames) {
            poseEst.addHeadingData(frame.timestampSecs(), frame.heading());
        }
        return update();
    }

    /** Fetches the latest pose estimates from this camera. */
    @SuppressWarnings("StringConcatenationInLoop")
    public List<CamPoseEstimate> update() {
        Tracer.startTrace("Vision Update (" + consts.name() + ")");

        Tracer.trace("Poll Data", () -> {
            io.refreshData(inputs);
            Logger.processInputs(key(""), inputs);
        });
        if (RobotMode.get() == RobotMode.REAL && !inputs.connected) {
            return List.of();
        }

        // process vision data into vision updates
        var poseEstimates = new ArrayList<CamPoseEstimate>();
        int ambHighCount = 0;
        int errHighCount = 0;
        String estimationStrat = "";
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
                fiducialIds.add(target.fiducialId);
            }
            if (ambiguityExceeded) {
                ambHighCount++;
                continue;
            }
            // updates the pose, and makes sure that the estimated pose
            // has a z coordinate near 0 and x and y coordinates within the field.
            var poseEstimate = poseEst.update(result);
            if (poseEstimate.isEmpty()) continue;
            estimationStrat += (poseEstimate.get().strategy + ",");
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
            poses.add(pose);
            var stdDevs = VecBuilder.fill(linearStdDev, linearStdDev, ANGULAR_STD_DEV);
            poseEstimates.add(new CamPoseEstimate(pose.toPose2d(), timestamp, stdDevs));
        }

        // logs relevant data
        boolean shouldLog = !inputs.results.isEmpty() && !DriverStation.isFMSAttached();
        if (RobotMode.get() == RobotMode.REPLAY || shouldLog) {
            int[] ids = new int[fiducialIds.size()];
            for (int i = 0; i < fiducialIds.size(); i++) {
                ids[i] = fiducialIds.get(i);
            }
            Logger.recordOutput(key("AprilTagIds"), ids);
            Logger.recordOutput(key("NumAmbiguityExceeded"), ambHighCount);
            Logger.recordOutput(key("NumErrExceeded"), errHighCount);
            Logger.recordOutput(key("Poses"), poses.toArray(new Pose3d[0]));
            Logger.recordOutput(key("EstimationStrategy"), estimationStrat);
        }

        Tracer.endTrace();
        return poseEstimates;
    }
}