package lib.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import lib.RobotMode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/** A class that batch refreshes CTRE status signals to improve performance. */
public class SignalRefresh {
    private static final HashMap<CANBus, ArrayList<BaseStatusSignal>> allSignals = new HashMap<>();

    /**
     * Adds signals to the signal refresher.
     * This system exists because calling refreshAll() for multiple signals is faster
     * than calling refresh() for each signal.
     * @param updateFreqHz 100hz (update once every 0.01 secs) is a good number to use.
     * @param canBus The network that the signals belong to. This can easily be accessed
     *               through the getNetwork() method of the CTRE device these signals come from.
     */
    public static void register(double updateFreqHz, CANBus canBus, BaseStatusSignal... signals) {
        var signalList = allSignals.computeIfAbsent(canBus, k -> new ArrayList<>());
        signalList.addAll(List.of(signals));
        BaseStatusSignal.setUpdateFrequencyForAll(updateFreqHz, signals);
    }

    /** Refreshes all signals. This must be called in robotPeriodic(). */
    public static void refreshAll() {
        if (RobotMode.get() == RobotMode.REPLAY) return;
        for (var group: allSignals.values()) {
            if (!group.isEmpty()) BaseStatusSignal.refreshAll(group);
        }
    }
}
