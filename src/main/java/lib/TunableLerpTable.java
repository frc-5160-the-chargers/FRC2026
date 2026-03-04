package lib;

import java.util.ArrayList;
import java.util.Collections;

/**
 * A linear interpolation (or "lerp" table) that can be tuned from the dashboard. <br />
 *
 * Imagine if you want to approximate a relationship between the distance to a shooting target
 * (x) with the optimal flywheel speed (y). <br />
 *
 * It's probably impossible to know the exact function that relates the 2. However, you can try
 * spinning the flywheel (and shooting balls) into the target at various distances. <br />
 *
 * Then, you could plot each of those points, and draw a line in between each of them. This effectively
 * creates an approximation of the actual function that relates x and y. That's basically it.
 */
public class TunableLerpTable {
    private record Entry(Tunable<Double> key, Tunable<Double> value) implements Comparable<Entry> {
        @Override
        public int compareTo(Entry other) {
            return Double.compare(this.key.get(), other.key.get());
        }
    }

    private final ArrayList<Entry> entries = new ArrayList<>();
    private final String name;

    public TunableLerpTable(String name) {
        this.name = name;
        // Since AdvantageScope displays tunable booleans as a red/green button,
        // We ignore the actual boolean value and just use it as a toggle.
        Tunable.of(name + "/Add Entry Button", false)
            .onChange(() -> {
                var last = entries.get(entries.size() - 1);
                put(last.key.get(), last.value.get());
            });
    }

    /** Adds an entry for linear interpolation. */
    public TunableLerpTable put(double key, double value) {
        String prefix = name + "/" + entries.size() + "/";
        var x = Tunable.of(prefix + "key", key);
        var y = Tunable.of(prefix + "value", value);
        x.onChange(() -> Collections.sort(entries));
        y.onChange(() -> Collections.sort(entries));
        entries.add(new Entry(x, y)); // ensures that values are sorted.
        Collections.sort(entries);
        return this;
    }

    public double get(double key) {
        var first = entries.get(0);
        var last = entries.get(entries.size() - 1);
        if (key <= first.key.get()) {
            return first.value.get();
        } else if (key >= last.key.get()) {
            return last.value.get();
        }
        for (int i = 0; i < entries.size() - 1; i++) {
            var e1 = entries.get(i);
            var e2 = entries.get(i + 1);
            double lowerBound = e1.key.get();
            double upperBound = e2.key.get();
            if (key >= lowerBound && key <= upperBound) {
                double proportion = (key - lowerBound) / (upperBound - lowerBound);
                return e1.value.get() + proportion * (e2.value.get() - e1.value.get());
            }
        }
        return 0;
    }

    public double getKey(double value) {
        var first = entries.get(0);
        var last = entries.get(entries.size() - 1);
        if (value <= first.value.get()) {
            return first.key.get();
        } else if (value >= last.value.get()) {
            return last.key.get();
        }
        for (int i = 0; i < entries.size() - 1; i++) {
            var e1 = entries.get(i);
            var e2 = entries.get(i + 1);
            double lowerBound = e1.value.get();
            double upperBound = e2.value.get();
            if (value >= lowerBound && value <= upperBound) {
                double proportion = (value - lowerBound) / (upperBound - lowerBound);
                return e1.key.get() + proportion * (e2.key.get() - e1.key.get());
            }
        }
        return 0;
    }
}
