package robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import robot.vision.Structs.MLCamConsts;

import java.util.Map;
import java.util.function.Supplier;

import static robot.vision.VisionConsts.DEFAULT_CAM_PROPERTIES;

/**
 * A variant of {@link CameraIO} for simulated object detection.
 * Uses the simulated game piece positions from maplesim as targets; to visualize these,
 * log <code>SimulatedArena.getInstance().getGamePieceArrayByType(String)</code>
 */
public class SimCameraIOForObjects extends CameraIO {
    private final VisionSystemSim sim;
    private final Map<String, TargetModel> availableObjects;
    private final Supplier<Pose2d> truePose;

    public SimCameraIOForObjects(MLCamConsts consts, Supplier<Pose2d> truePose) {
        super(consts.name());
        this.sim = new VisionSystemSim(consts.name());
        this.availableObjects = consts.availableObjects();
        this.truePose = truePose;
        var properties = DEFAULT_CAM_PROPERTIES.copy();
        properties.setCalibration(1280, 720, Rotation2d.fromDegrees(60));
        sim.addCamera(
            new PhotonCameraSim(cam, properties, 0.12, 4),
            consts.robotCenterToCamera()
        );
    }

    @Override
    public void refreshData(CameraIO.RawData inputs) {
        for (var data: availableObjects.entrySet()) {
            String type = data.getKey();
            TargetModel model = data.getValue();
            sim.removeVisionTargets(type);
            var poses = SimulatedArena.getInstance().getGamePiecesPosesByType(type);
            if (poses.isEmpty()) {
                DriverStation.reportError(
                    "No Game Pieces in MapleSim of type " + type + "; are you sure you spelled it correctly?",
                    false
                );
            }
            for (var pose: poses) {
                sim.addVisionTargets(type, new VisionTargetSim(pose, model));
            }
        }
        sim.update(truePose.get());
        super.refreshData(inputs);
    }
}
