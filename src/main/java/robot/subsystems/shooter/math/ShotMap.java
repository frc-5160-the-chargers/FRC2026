package robot.subsystems.shooter.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.Alert;
import robot.subsystems.shooter.DataTypes.ShotMapResult;

import java.util.Map;
import java.util.TreeMap;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

public class ShotMap {
    // t is a fraction between 0 and 1.
    private static ShotMapResult interpolateShot(ShotMapResult start, ShotMapResult end, double t) {
        return new ShotMapResult(
            start.pitchRad() + (end.pitchRad() - start.pitchRad()) * t,
            start.airTimeSecs() + (end.airTimeSecs() - start.airTimeSecs()) * t
        );
    }

    private final TreeMap<Double, InterpolatingTreeMap<Double, ShotMapResult>> resultsMap = new TreeMap<>();
    private final InterpolatingDoubleTreeMap
        distanceToMinVelocity = new InterpolatingDoubleTreeMap(),
        distanceToMaxVelocity = new InterpolatingDoubleTreeMap();

    /**
     * Adds a group of velocity to ideal shooter hood angle mappings
     * that are valid at a certain distance away from the target.
     * @param distanceMeters The distance from the shooter's center to the target.
     * @param entries The velocity to hood angle mappings.
     */
    @SafeVarargs
    public final void put(double distanceMeters, Map.Entry<Double, ShotMapResult>... entries) {
        if (entries.length == 0) {
            new Alert("ShotMap#put called with zero entries", kError).set(true);
            return;
        }
        var speedToAngleMap = new InterpolatingTreeMap<>(MathUtil::inverseInterpolate, ShotMap::interpolateShot);
        for (var entry : entries) {
            speedToAngleMap.put(entry.getKey(), entry.getValue());
        }
        resultsMap.put(distanceMeters, speedToAngleMap);
        distanceToMinVelocity.put(distanceMeters, entries[0].getKey());
        distanceToMaxVelocity.put(distanceMeters, entries[entries.length - 1].getKey());
    }

    /**
     * Fetches the shooter hood angle necessary to score a ball,
     * as well as the total time that ball will take to score,
     * given the shooter's distance to the target and the wanted speed of the ball.
     * @param distanceM The current distance from the shooter to the target.
     * @param speedMPS The wanted speed of the ball in meters/sec. Must be positive.
     * @return Shooter hood angle + airtime.
     */
    public ShotMapResult get(double distanceM, double speedMPS) {
        var val = resultsMap.get(distanceM);
        if (val != null) return val.get(speedMPS);

        var lowerBoundDistance = resultsMap.floorKey(distanceM);
        var upperBoundDistance = resultsMap.ceilingKey(distanceM);

        if (lowerBoundDistance == null && upperBoundDistance == null) return null;
        if (lowerBoundDistance == null) {
            return resultsMap.get(upperBoundDistance).get(speedMPS);
        }
        if (upperBoundDistance == null){
            return resultsMap.get(lowerBoundDistance).get(speedMPS);
        }

        double fractionalDiff = (distanceM - lowerBoundDistance) / (upperBoundDistance - lowerBoundDistance);
        return interpolateShot(
            resultsMap.get(lowerBoundDistance).get(speedMPS),
            resultsMap.get(upperBoundDistance).get(speedMPS),
            fractionalDiff
        );
    }

    /**
     * Gets the derivative of the air time with respect to the distance traveled,
     * while air time stays constant.
     * Imagine if you chose a random velocity (let's say 5 m/s), then
     * plotted air time on the y axis and distance travelled on the x axis.
     * The derivative would be the slope of the curve at the specific distance
     * you specified.
     */
    public double getAirTimeDerivative(double distanceM, double speedMPS) {
        double airTimeUpperBound = get(distanceM + 0.01, speedMPS).airTimeSecs();
        double airTimeLowerBound = get(distanceM - 0.01, speedMPS).airTimeSecs();
        return (airTimeUpperBound - airTimeLowerBound) / 0.02;
    }

    /** Fetches the max velocity of the shooter at the given distance. */
    public double maxVelocityAt(double distanceM) {
        return distanceToMaxVelocity.get(distanceM);
    }

    /** Fetches the min velocity of the shooter at the given distance. */
    public double minVelocityAt(double distanceM) {
        return distanceToMaxVelocity.get(distanceM);
    }
}