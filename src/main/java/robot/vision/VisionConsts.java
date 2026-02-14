package robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import lib.Convert;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.SimCameraProperties;
import robot.subsystems.drive.TunerConstants;
import robot.vision.DataTypes.AprilTagCamConsts;
import robot.vision.DataTypes.MLCamConsts;

import java.util.Map;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class VisionConsts {
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(
        AprilTagFields.k2026RebuiltAndymark
    );
    public static final AprilTagFieldLayout REEF_ONLY_LAYOUT = new AprilTagFieldLayout(
        FIELD_LAYOUT.getTags()
            .stream()
            .filter(it -> (it.ID >= 17 && it.ID <= 22) || (it.ID >= 6 && it.ID <= 11))
            .toList(),
        FIELD_LAYOUT.getFieldLength(),
        FIELD_LAYOUT.getFieldWidth()
    );

    public static final AprilTagCamConsts FL_CONSTS = new AprilTagCamConsts(
        "Chargers-FrontLeft",
        new Transform3d(
            Meters.of(TunerConstants.FrontLeft.LocationX - 0.026),
            Meters.of(TunerConstants.FrontLeft.LocationY + 0.11),
            Inches.of(7.375),
            new Rotation3d(
                Degrees.zero(),
                Degrees.of(-15),
                Degrees.of(-46) // measured as: 46, previously working: 48
            )
        ),
        REEF_ONLY_LAYOUT, 1.0, Optional.empty()
    );
    public static final AprilTagCamConsts FR_CONSTS = new AprilTagCamConsts(
        "Chargers-FrontRight",
        new Transform3d(
            Meters.of(TunerConstants.FrontLeft.LocationX - 0.026),
            Meters.of(TunerConstants.FrontLeft.LocationY - 0.1),
            Inches.of(7.375),
            new Rotation3d(
                Degrees.zero(),
                Degrees.of(-15),
                Degrees.of(-48) // prob correct - another value -56
            )
        ),
        REEF_ONLY_LAYOUT, 1.0, Optional.empty()
    );
    public static final MLCamConsts CORAL_CAM_CONSTS = new MLCamConsts(
        "CoralCam",
        Map.of(
            "Coral",
            new TargetModel(
                11.875 * Convert.INCHES_TO_METERS,
                4.5 * Convert.INCHES_TO_METERS,
                4.5 * Convert.INCHES_TO_METERS
            )
        ),
        new Transform3d(
            new Translation3d(-0.3, 0, 0.254),
            new Rotation3d(0, 0, 180 * Convert.DEGREES_TO_RADIANS)
        ),
        Optional.empty()
    );

    public static final double MAX_AMBIGUITY = 0.2;
    public static final Distance MAX_Z_ERROR = Meters.of(0.1);
    public static final double Z_ERROR_SCALAR = 100.0;
    public static final double SINGLE_TAG_SCALAR = 1.3;
    public static final double LINEAR_STD_DEV_BASELINE = 0.3;
    public static final double ANGULAR_STD_DEV = 10000000;
    public static final SimCameraProperties DEFAULT_CAM_PROPERTIES = new SimCameraProperties();

    static {
        DEFAULT_CAM_PROPERTIES.setCalibError(0.25, 0.15);
        DEFAULT_CAM_PROPERTIES.setAvgLatencyMs(35);
        DEFAULT_CAM_PROPERTIES.setLatencyStdDevMs(5);
        DEFAULT_CAM_PROPERTIES.setFPS(40);
    }
}
