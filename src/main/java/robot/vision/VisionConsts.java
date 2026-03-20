package robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import lib.Convert;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.SimCameraProperties;
import robot.subsystems.drive.TunerConstants;
import robot.vision.DataTypes.AprilTagCamConsts;
import robot.vision.DataTypes.MLCamConsts;

import java.io.File;
import java.io.IOException;
import java.util.Map;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class VisionConsts {
    public enum AprilTagPipeline {
        HOME(0),
        WAKE_WEEK_2(1);

        public final int index;

        AprilTagPipeline(int index) {
            this.index = index;
        }
    }

    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(
        AprilTagFields.k2026RebuiltAndymark
    );

    // GOOD
    public static final AprilTagCamConsts FL_CONSTS = new AprilTagCamConsts(
        "Chargers-1",
        new Transform3d(
            Inches.of(6.351).plus(Meters.of(0.01)),
            Inches.of(12.136),
            Inches.of(20.715).minus(Meters.of(0.03)),
            new Rotation3d(
                Degrees.zero(),
                Degrees.of(-15),
                Degrees.of(-20)
            )
        ),
        FIELD_LAYOUT, 1.0, Optional.empty()
    );

    // GOOD
    public static final AprilTagCamConsts FR_CONSTS = new AprilTagCamConsts(
        "Chargers-2",
        new Transform3d(
            Inches.of(6.1).minus(Meters.of(0.01)),
            Inches.of(-12.228),
            Inches.of(20.8).minus(Meters.of(0.05)),
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
    public static final Distance MAX_Z_ERROR = Meters.of(0.25);
    public static final double Z_ERROR_SCALAR = 30.0;
    public static final double SINGLE_TAG_SCALAR = 2.0;
    public static final double LINEAR_STD_DEV_BASELINE = 0.22;
    public static final double ANGULAR_STD_DEV = 2.5;
    public static final SimCameraProperties DEFAULT_CAM_PROPERTIES = new SimCameraProperties();

    static {
        DEFAULT_CAM_PROPERTIES.setCalibError(0.25, 0.15);
        DEFAULT_CAM_PROPERTIES.setAvgLatencyMs(35);
        DEFAULT_CAM_PROPERTIES.setLatencyStdDevMs(5);
        DEFAULT_CAM_PROPERTIES.setFPS(40);
    }
}
