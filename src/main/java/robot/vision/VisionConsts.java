package robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import lib.Tunable;
import org.photonvision.simulation.SimCameraProperties;
import robot.vision.DataTypes.AprilTagCamConsts;

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

    public static final AprilTagFieldLayout FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

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

    static final Tunable<Double>
        MAX_AMBIGUITY = Tunable.of("Vision/MaxAmbiguity", 0.2),
        Z_ERROR_SCALAR = Tunable.of("Vision/ZErrorScalar", 30.0),
        SINGLE_TAG_SCALAR = Tunable.of("Vision/SingleTagScalar", 2.0),
        LINEAR_STD_DEV_BASELINE = Tunable.of("Vision/StdDevBaseline/Linear", 0.22),
        ANGULAR_STD_DEV_BASELINE = Tunable.of("Vision/StdDevBaseline/Angular", 2.5);
    static final Tunable<Distance> MAX_Z_ERROR = Tunable.of("Vision/MaxZErr", Meters.of(0.25));
    static final Tunable<Boolean> DEBUG_MODE = Tunable.of("Vision/DebugMode", false);
    static final SimCameraProperties DEFAULT_CAM_PROPERTIES = new SimCameraProperties();

    static {
        DEFAULT_CAM_PROPERTIES.setCalibError(0.25, 0.15);
        DEFAULT_CAM_PROPERTIES.setAvgLatencyMs(35);
        DEFAULT_CAM_PROPERTIES.setLatencyStdDevMs(5);
        DEFAULT_CAM_PROPERTIES.setFPS(40);
    }
}
