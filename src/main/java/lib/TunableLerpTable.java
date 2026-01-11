package lib;

import java.util.ArrayList;
import java.util.Collections;

/** A linear interpolation table with tuning support. */
public class TunableLerpTable {
    private record Entry(Tunable<Double> x, Tunable<Double> y) implements Comparable<Entry> {
        public double interpolate(Entry other, double x) {
            double thisY = this.y.get();
            double thisX = this.x.get();
            double proportion = (other.y.get() - thisY) / (other.x.get() - thisX);
            return thisY + proportion * (x - thisX);
        }

        @Override
        public int compareTo(Entry other) {
            return Double.compare(this.x.get(), other.x.get());
        }
    }

    private final ArrayList<Entry> entries = new ArrayList<>();
    private final String name, xName, yName;

    public TunableLerpTable(String xName, String yName) {
        this.name = "Linear Interpolation/" + xName + " to " + yName + "/";
        this.xName = xName;
        this.yName = yName;
        // Since AdvantageScope displays tunable booleans as a red/green button,
        // We ignore the actual boolean value and just use it as a toggle.
        Tunable.of(name + "/Add Entry Button", false)
            .onChange(ignore -> {
                var last = entries.get(entries.size() - 1);
                put(last.x.get(), last.y.get());
            });
    }

    /** Adds an entry for linear interpolation. */
    public TunableLerpTable put(double key, double value) {
        String prefix = name + "/" + entries.size() + "/";
        var x = Tunable.of(prefix + xName, key);
        var y = Tunable.of(prefix + yName, value);
        entries.add(new Entry(x, y));
        Collections.sort(entries);
        return this;
    }

    /** Fetches a number from the interpolation table. */
    public double get(double x) {
        var first = entries.get(0);
        var last = entries.get(entries.size() - 1);
        if (x <= first.x.get()) {
            return first.y.get();
        } else if (x >= last.x.get()) {
            return last.y.get();
        }
        for (int i = 0; i < entries.size() - 1; i++) {
            var e1 = entries.get(i);
            var e2 = entries.get(i + 1);
            if (x >= e1.x.get() && x <= e2.x.get()) {
                return e1.interpolate(e2, x);
            }
        }
        return 0;
    }
}
