package robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values
 * in the Choreo GUI.
 */
public final class ChoreoVars {
    public static final Mass botMass = Kilograms.of(63.5);
    public static final MomentOfInertia botMoi = KilogramSquareMeters.of(5.8);
    public static final double cof = 1.5;
    public static final double gearing = 6.2;

    private ChoreoVars() {}
}