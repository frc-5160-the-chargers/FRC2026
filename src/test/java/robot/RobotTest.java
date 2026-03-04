package robot;

import lib.RobotMode;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LoggedRobot;
import testingutil.ChargerUnitTest;

import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class RobotTest extends ChargerUnitTest {
    @Test
    public void instantiateRobot() {
        RobotMode.setShimRealRobot(true);
        var robot = new Robot();
        instantiate(robot);
        try {
            var simArena = SimulatedArena.class.getDeclaredField("instance");
            simArena.setAccessible(true);
            assertNull(
                simArena.get(null),
                "MapleSim should only be instantiated after checking that RobotMode.isSim() returns true."
            );
        } catch (NoSuchFieldException | IllegalAccessException ignored) {}
        robot.close();
    }

    private void instantiate(LoggedRobot robot) {
        // Init robot
        robot.robotInit();

        // Start with the robot disabled
        robot.disabledInit();
        robot.disabledPeriodic();
        robot.robotPeriodic();
        robot.disabledExit();

        // Enable auto
        robot.autonomousInit();
        robot.autonomousPeriodic();
        robot.robotPeriodic();
        robot.autonomousExit();

        // Disable
        robot.disabledInit();
        robot.disabledPeriodic();
        robot.robotPeriodic();
        robot.disabledExit();

        // Enable teleop
        robot.teleopInit();
        robot.teleopPeriodic();
        robot.robotPeriodic();
        robot.teleopExit();

        // Disable
        robot.disabledInit();
        robot.disabledPeriodic();
        robot.robotPeriodic();
        robot.disabledExit();

        // Enable test
        robot.testInit();
        robot.testPeriodic();
        robot.robotPeriodic();
        robot.testExit();

        // Disable
        robot.disabledInit();
        robot.disabledPeriodic();
        robot.robotPeriodic();
    }
}