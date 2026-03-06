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

    // Long: 12.137 left, 5.716 forward, 21.663 up
    // Short: 12.008 left, 5.689 forward, 21.602 up

    // assumes that front right cam is the 15 deg pitch one,
    // front left cam is the 25 deg pitch one

    public static final AprilTagCamConsts FL_CONSTS = new AprilTagCamConsts(
        "Chargers-2",
        new Transform3d(
            Inches.of(5.689),
            Inches.of(12.008),
            Inches.of(21.602),
            new Rotation3d(
                Degrees.zero(),
                Degrees.of(-15),
                Degrees.of(-20)
            )
        ),
        FIELD_LAYOUT, 1.0, Optional.empty()
    );
    public static final AprilTagCamConsts FR_CONSTS = new AprilTagCamConsts(
        "Chargers-1",
        new Transform3d(
            Inches.of(5.716),
            Inches.of(-12.137),
            Inches.of(21.663),
            new Rotation3d(
                Degrees.zero(),
                Degrees.of(-25),
                Degrees.of(20)
            )
        ),
        FIELD_LAYOUT, 1.0, Optional.empty()
    );

    // 9.5 back, 17.15 up

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
