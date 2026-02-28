package lib;


import edu.wpi.first.wpilibj.DriverStation;

public class AllianceColor {
    /** Returns whether the current alliance color is red. */
    public static boolean isRed() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }
}
