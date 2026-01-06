package lib;

import edu.wpi.first.wpilibj.RobotBase;
import lombok.Setter;

import java.util.Objects;

public enum RobotMode {
    REAL, SIM, REPLAY;

    // Running "./gradlew simulateJava -Preplay" will enable this.
    // (see line 127 in build.gradle)
    private static final boolean isReplay =
        Objects.equals(System.getProperty("replayMode"), "enabled");
    // Only use this for unit tests.
    @Setter private static boolean shimRealRobot = false;

    /** Fetches the current robot mode. */
    public static RobotMode get() {
        if (!RobotBase.isSimulation() || shimRealRobot) return REAL;
        return isReplay ? REPLAY : SIM;
    }

    /** A convenience method for checking if the current robot mode is simulation. */
    public static boolean isSim() {
        return get() == SIM;
    }
}
