package robot.constants;

import edu.wpi.first.wpilibj.DriverStation;

public class GlobalConsts {
    /** Returns true when the current alliance is red. */
    public static boolean redAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }
}
