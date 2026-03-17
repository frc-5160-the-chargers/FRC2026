package lib;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.Random;

public class LoggedRandomNum extends Random {
    private long seed;

    public LoggedRandomNum(String name) {
        seed = nextLong();
        var logHandle = new LoggableInputs() {
            @Override
            public void toLog(LogTable table) {
                table.put("RandomSeed", seed);
            }

            @Override
            public void fromLog(LogTable table) {
                seed = table.get("RandomSeed", 0L);
            }
        };
        Logger.processInputs(name, logHandle);
        super.setSeed(seed);
    }
}
