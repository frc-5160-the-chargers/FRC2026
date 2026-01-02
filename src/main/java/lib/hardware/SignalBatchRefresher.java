package lib.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import lib.RobotMode;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;

/** A class that batch refreshes CTRE status signals to improve performance. */
public class SignalBatchRefresher {
    private static final HashMap<CANBus, ArrayList<BaseStatusSignal>> allSignals = new HashMap<>();

    /**
     * Adds signals to the signal refresher.
     * This system exists because calling refreshAll() for multiple signals is faster
     * than calling refresh() for each signal.
     * @param canBus The network that the signals belong to. This can easily be accessed
     *               through the getNetwork() method of the CTRE device these signals come from.
     */
    public static void register(CANBus canBus, Collection<BaseStatusSignal> signals) {
        var signalList = allSignals.computeIfAbsent(canBus, k -> new ArrayList<>());
        signalList.addAll(signals);
    }

    /** Refreshes all signals. This must be called in robotPeriodic(). */
    public static void refreshAll() {
        if (RobotMode.get() == RobotMode.REPLAY) return;
        for (var group: allSignals.values()) {
            if (!group.isEmpty()) BaseStatusSignal.refreshAll(group);
        }
    }
}
