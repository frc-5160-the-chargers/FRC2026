package robot.subsystems.climber;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberData implements LoggableInputs{
    public double volts = 0.0;
    public double pos = 0;

    @Override
    public void toLog(LogTable table) {
        table.put("volts", volts);
        table.put("pos", pos);
    }

    // Used to replace the values we saved from a previous run
    // so we can treat it as though the robot is going through
    // the same circumstances again
    @Override
    public void fromLog(LogTable table) {
        volts = table.get("volts", 0.0);
        pos = table.get("pos", 0.0);
    }
}
