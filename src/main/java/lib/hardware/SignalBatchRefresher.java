package lib.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.ParentDevice;
import lib.RobotMode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/** A class that batch refreshes CTRE status signals to improve performance. */
public class SignalBatchRefresher {
    private static final HashMap<CANBus, List<BaseStatusSignal>> allSignals = new HashMap<>();

    /**
     * Adds signals to the signal refresher.
     * This system exists because calling refreshAll() for multiple signals is faster
     * than calling refresh() for each signal.
     * @param device The device that the signals belong to. For instance, a
     *               {@link com.ctre.phoenix6.hardware.CANcoder}.
     */
    public static void register(ParentDevice device, List<BaseStatusSignal> signals) {
        var signalList = allSignals.computeIfAbsent(device.getNetwork(), k -> new ArrayList<>());
        signalList.addAll(signals);
    }

    /**
     * Refreshes all signals. This must be called in robotPeriodic().
     */
    public static void refreshAll() {
        if (RobotMode.get() == RobotMode.REPLAY) return;
        for (var group: allSignals.values()) {
            if (!group.isEmpty()) BaseStatusSignal.refreshAll(group);
        }
    }
}
