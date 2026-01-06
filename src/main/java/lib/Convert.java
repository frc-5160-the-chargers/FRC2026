package lib;

/**
 * A compilation of useful unit conversion factors.
 * A couple of notes:
 * 1. Positions fetched from motors are always in rotations.
 * 2. Velocities for CTRE motors are in rotations/sec, while for REV motors,
 * they are always in RPM (rotations/minute).
 */
public class Convert {
    public static final double FEET_TO_METERS = 0.3048;
    public static final double INCHES_TO_METERS = 0.0254;
    public static final double METERS_TO_INCHES = 1.0 / INCHES_TO_METERS;
    public static final double DEGREES_TO_RADIANS = Math.PI / 180.0;
    public static final double ROTATIONS_TO_RADIANS = 2 * Math.PI;
    public static final double RPM_TO_RADIANS_PER_SECOND = 2 * Math.PI / 60.0;
    public static final double RADIANS_TO_DEGREES = 180.0 / Math.PI;
    public static final double RADIANS_TO_ROTATIONS = 1.0 / (2 * Math.PI);
}
