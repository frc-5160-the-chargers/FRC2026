package robot.subsystems.shooter.math;

import edu.wpi.first.math.geometry.Translation2d;
import robot.misc.SharedData;

import static choreo.util.ChoreoAllianceFlipUtil.flip;

public record ShotLocation(Translation2d blueAlliancePos, ShotMap shotMap) {
    public Translation2d position() {
        return SharedData.redAlliance() ? flip(blueAlliancePos) : blueAlliancePos;
    }

//    public static final ShotLocation HUB = new ShotLocation(...);
//    public static final ShotLocation FERRY = new ShotLocation(...);
}
