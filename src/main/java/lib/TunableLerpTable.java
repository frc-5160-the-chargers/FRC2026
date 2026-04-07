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
    private static class Entry implements Comparable<Entry> {
        public double key, value;
        
        @Override
        public int compareTo(Entry other) {
            return Double.compare(this.key, other.key);
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
                put(last.key, last.value);
            });
    }

    /** Adds an entry for linear interpolation. */
    public TunableLerpTable put(double key, double value) {
        var entry = new Entry();
        entry.key = key;
        entry.value = value;
        entries.add(entry);
        Collections.sort(entries);
        var prefix = name + "/" + entries.size() + "/";
        Tunable.of(prefix + "key", key)
            .onChange(newKey -> {
                entry.key = newKey;
                Collections.sort(entries);
            });
        Tunable.of(prefix + "value", value)
            .onChange(newValue -> entry.value = newValue);
        return this;
    }

    public double get(double key) {
        var first = entries.get(0);
        var last = entries.get(entries.size() - 1);
        if (key <= first.key) {
            return first.value;
        } else if (key >= last.key) {
            return last.value;
        }
        for (int i = 0; i < entries.size() - 1; i++) {
            var e1 = entries.get(i);
            var e2 = entries.get(i + 1);
            double lowerBound = e1.key;
            double upperBound = e2.key;
            if (key >= lowerBound && key <= upperBound) {
                double proportion = (key - lowerBound) / (upperBound - lowerBound);
                return e1.value + proportion * (e2.value - e1.value);
            }
        }
        return 0;
    }

    public double getKey(double value) {
        var first = entries.get(0);
        var last = entries.get(entries.size() - 1);
        if (value <= first.value) {
            return first.key;
        } else if (value >= last.value) {
            return last.key;
        }
        for (int i = 0; i < entries.size() - 1; i++) {
            var e1 = entries.get(i);
            var e2 = entries.get(i + 1);
            double lowerBound = e1.value;
            double upperBound = e2.value;
            if (value >= lowerBound && value <= upperBound) {
                double proportion = (value - lowerBound) / (upperBound - lowerBound);
                return e1.key + proportion * (e2.key - e1.key);
            }
        }
        return 0;
    }
}
