package robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values
 * in the Choreo GUI.
 */
public final class ChoreoVars {
    public static final double cof = 1.5;
    public static final double gearing = 6.2;
    public static final Distance moduleX = Units.Meters.of(0.273);
    public static final Distance moduleY = Units.Meters.of(0.279);
    public static final MomentOfInertia robotMOI = Units.KilogramSquareMeters.of(5.8);
    public static final Mass robotMass = Units.Kilograms.of(63.5);
    public static final Distance wheelRadius = Units.Meters.of(0.048);

    public static final class Poses {
        public static final Pose2d CenterShotPos = new Pose2d(2.6, 4, Rotation2d.kZero);
        public static final Pose2d HubPos = new Pose2d(4.62, 4.05, Rotation2d.kZero);
        public static final Pose2d RightStartPos = new Pose2d(3.65, 0.9, Rotation2d.kZero);
        public static final Pose2d StationPickupPos = new Pose2d(0.77, 0.68, Rotation2d.kZero);

        private Poses() {}
    }

    private ChoreoVars() {}
}