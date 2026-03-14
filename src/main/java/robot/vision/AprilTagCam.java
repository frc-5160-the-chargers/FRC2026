package robot.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import lib.RobotMode;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import robot.vision.DataTypes.AprilTagCamConsts;
import robot.vision.DataTypes.CamPoseEstimate;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static edu.wpi.first.units.Units.Meters;
import static robot.vision.VisionConsts.*;

/** Handles processing pose estimates from an apriltag camera. */
public class AprilTagCam {
    private final AprilTagCamConsts consts;
    private final PhotonPoseEstimator poseEst;
    private final CameraIO io;
    private final CameraIO.RawData inputs = new CameraIO.RawData();
    private final Alert disconnectedAlert;

    private final Set<Integer> fiducialIds = new HashSet<>();
    private final List<Pose3d> poses = new ArrayList<>();

    public AprilTagCam(SwerveDriveSimulation swerveSim, AprilTagCamConsts consts) {
        this.consts = consts;
        disconnectedAlert = new Alert("Camera " + consts.name() + " is disconnected!", Alert.AlertType.kError);
        this.io = RobotMode.isSim()
            ? new SimCameraIOForTags(swerveSim::getSimulatedDriveTrainPose, consts)
            : new CameraIO(consts.name());
        this.poseEst = new PhotonPoseEstimator(
            consts.fieldLayout(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            consts.robotCenterToCamera()
        );
        Logger.recordOutput(key("CamRelativePos"), consts.robotCenterToCamera());
    }
    
    private String key(String path) {
        return "Cameras/" + consts.name() + "/" + path;
    }

    /** Fetches the latest pose estimates from this camera. */
    @SuppressWarnings("StringConcatenationInLoop")
    public List<CamPoseEstimate> update() {
        io.refreshData(inputs);
        Logger.processInputs(key(""), inputs);
        disconnectedAlert.set(!inputs.connected);
        if (RobotMode.get() == RobotMode.REAL && !inputs.connected) {
            return List.of();
        }

        // process vision data into vision updates
        // process vision data into esvision updates
        var poseEstimates = new ArrayList<CamPoseEstimate>();
        int ambHighCount = 0;
        int errHighCount = 0;
        String estimationStrat = "";
        fiducialIds.clear();
        poses.clear();
        Logger.recordOutput(key("NumResults"), inputs.results.size());
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
        if (shouldLog || RobotMode.get() == RobotMode.REPLAY) {
            int[] fiducialIdsArray = new int[fiducialIds.size()];
            int i = 0;
            for (var id: fiducialIds) {
                fiducialIdsArray[i] = id;
                i++;
            }
            Logger.recordOutput(key("AprilTagIds"), fiducialIdsArray);
            Logger.recordOutput(key("NumAmbiguityExceeded"), ambHighCount);
            Logger.recordOutput(key("NumErrExceeded"), errHighCount);
            Logger.recordOutput(key("Poses"), poses.toArray(new Pose3d[0]));
            Logger.recordOutput(key("EstimationStrategy"), estimationStrat);
        }

        return poseEstimates;
    }

    /** Performs a data update without computing anything. */
    public void updateWithoutCompute() {
        io.refreshData(inputs);
        Logger.processInputs(key(""), inputs);
    }
}