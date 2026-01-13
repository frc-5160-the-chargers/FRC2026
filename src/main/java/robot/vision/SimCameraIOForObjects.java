package robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import lib.Tracer;
import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import robot.vision.DataTypes.MLCamConsts;

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
    private final Supplier<Pose2d> robotTruePose;

    public SimCameraIOForObjects(MLCamConsts consts, Supplier<Pose2d> truePose) {
        super(consts.name());
        sim = new VisionSystemSim(consts.name());
        availableObjects = consts.availableObjects();
        robotTruePose = truePose;
        var properties = DEFAULT_CAM_PROPERTIES.copy();
        if (consts.intrinsics().isPresent()) {
            var i = consts.intrinsics().get();
            properties.setCalibration(i.width(), i.height(), i.cameraMatrix(), i.distortionMatrix());
        } else {
            properties.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
        }
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
        Tracer.trace("Simulation", () -> sim.update(robotTruePose.get()));
        super.refreshData(inputs);
    }
}
