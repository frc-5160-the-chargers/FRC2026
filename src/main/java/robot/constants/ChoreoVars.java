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
    public static final MomentOfInertia botMOI = Units.KilogramSquareMeters.of(5.8);
    public static final Mass botMass = Units.Kilograms.of(63.5);
    public static final double cof = 1.5;
    public static final double gearing = 6.2;
    public static final Distance moduleX = Units.Meters.of(0.348);
    public static final Distance moduleY = Units.Meters.of(0.279);
    public static final double moi = 1;
    public static final Distance wheelRadius = Units.Meters.of(0.051);

    private ChoreoVars() {}
}