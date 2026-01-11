package lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A tunable value. Can be found in the "Tuning" section of AdvantageScope.
 * Will be disabled if ```Tunable.setEnabled(false)``` is called; if that is
 * undesirable, use {@link LoggedNetworkNumber} or {@link LoggedNetworkBoolean}.
 */
public class Tunable<T> extends LoggedNetworkInput {
    /** When Tunable.setEnabled(true) is called, dashboard tuning will be enabled. */
    @Setter private static boolean enabled = false;

    /** Creates a tunable double. */
    public static Tunable<Double> of(String key, double value) {
        var ntHandle = new LoggedNetworkNumber("/Tuning/" + key, value);
        return new Tunable<>(ntHandle::get, value);
    }

    /** Creates a tunable boolean. */
    public static Tunable<Boolean> of(String key, boolean value) {
        var ntHandle = new LoggedNetworkBoolean("/Tuning/" + key, value);
        return new Tunable<>(ntHandle::get, value);
    }

    /** Creates a tunable measure. */
    @SuppressWarnings("unchecked")
    public static <M extends Measure<?>> Tunable<M> of(String key, M value) {
        String fullKey = "/Tuning/" + key + "(" + value.unit().name() + ")";
        var ntHandle = new LoggedNetworkNumber(fullKey, value.magnitude());
        return new Tunable<>(() -> (M) value.unit().of(ntHandle.get()), value);
    }

    /** Creates a tunable {@link Pose2d}. */
    public static Tunable<Pose2d> of(String key, Pose2d value) {
        var x = new LoggedNetworkNumber(key + "/xMeters", value.getX());
        var y = new LoggedNetworkNumber(key + "/yMeters", value.getY());
        var rot = new LoggedNetworkNumber(
            key + "/headingDeg", value.getRotation().getDegrees()
        );
        return new Tunable<>(
            () -> new Pose2d(x.get(), y.get(), Rotation2d.fromDegrees(rot.get())),
            value
        );
    }

    private T value;
    private final Supplier<T> supplier;
    private final ArrayList<Consumer<T>> listeners = new ArrayList<>();

    private Tunable(Supplier<T> supplier, T value) {
        this.value = value;
        this.supplier = supplier;
        Logger.registerDashboardInput(this);
    }

    /** Adds a function that listens to changes to this tunable value. */
    public void onChange(Consumer<T> listener) {
        listeners.add(listener);
    }

    /** Fetches the value. */
    public T get() {
        return value;
    }

    @Override
    public void periodic() {
        if (!enabled || DriverStation.isFMSAttached()) return;
        T latest = supplier.get();
        if (!value.equals(latest)) {
            value = latest;
            for (var listener: listeners) {
                listener.accept(latest);
            }
        }
    }
}
