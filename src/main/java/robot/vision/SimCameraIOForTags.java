package robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import lib.Tracer;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import robot.SharedData;
import robot.vision.DataTypes.AprilTagCamConsts;

import static robot.vision.VisionConsts.DEFAULT_CAM_PROPERTIES;

/** A variant of {@link CameraIO} for simulated AprilTag detection. */
public class SimCameraIOForTags extends CameraIO {
    private final VisionSystemSim sim;

    public SimCameraIOForTags(AprilTagCamConsts consts) {
        super(consts.name());
        this.sim = new VisionSystemSim(consts.name());
        var properties = DEFAULT_CAM_PROPERTIES.copy();
        if (consts.intrinsics().isPresent()) {
            var i = consts.intrinsics().get();
            properties.setCalibration(i.width(), i.height(), i.cameraMatrix(), i.distortionMatrix());
        } else {
            properties.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
        }
        sim.addCamera(
            new PhotonCameraSim(super.cam, properties, 0.12, 6.5),
            consts.robotCenterToCamera()
        );
        sim.addAprilTags(consts.fieldLayout());
    }

    @Override
    public void refreshData(CameraIO.RawData data) {
        Tracer.trace("Simulation", () -> sim.update(SharedData.truePoseInSim));
        super.refreshData(data);
    }
}
