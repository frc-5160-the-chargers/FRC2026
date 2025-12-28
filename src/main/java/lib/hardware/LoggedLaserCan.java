package lib.hardware;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.GrappleJNI;
import au.grapplerobotics.LaserCan;
import org.littletonrobotics.junction.AutoLog;

/**
 * A LaserCan with replay support, by refreshing a {@link LaserCanDataAutoLogged}.
 */
public class LoggedLaserCan {
    static {
        // A patch to ensure that LaserCan data is functioning. Do not delete!
        GrappleJNI.forceLoad();
        CanBridge.runTCP();
    }

    /** Represents data received from a {@link LoggedLaserCan}. */
    @AutoLog
    public static class LaserCanData {
        public double distance_mm = -1;
        public int ambient = -1;
        public int budgetMs = -1;
        public boolean isLong = false;
        public int status = 0;

        public boolean isValid() {
            return status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
        }
    }
    
    private final LaserCan laserCan;

    public LoggedLaserCan(int canId) {
        this.laserCan = new LaserCan(canId);
    }

    public void refreshData(LaserCanDataAutoLogged inputs) {
        var measurement = laserCan.getMeasurement();
        if (measurement == null || measurement.status == -1) {
            inputs.status = -1;
            inputs.distance_mm = -1;
            inputs.ambient = -1;
            inputs.budgetMs = -1;
        } else {
            inputs.status = measurement.status;
            inputs.distance_mm = measurement.distance_mm;
            inputs.ambient = measurement.ambient;
            inputs.budgetMs = measurement.budget_ms;
            inputs.isLong = measurement.is_long;
        }
    }
}
