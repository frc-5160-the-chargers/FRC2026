package robot;

import lib.RobotMode;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LoggedRobot;
import testingutil.ChargerUnitTest;

import static org.junit.jupiter.api.Assertions.assertNull;

// TODO make this not create a new WPILOG file every run
class RobotTest extends ChargerUnitTest {
    @Test
    public void instantiateSimRobot() {
        var robot = new Robot();
        instantiate(robot);
        robot.close();
    }

    @Test
    public void instantiateRealRobot() throws NoSuchFieldException, IllegalAccessException {
        RobotMode.setShimRealRobot(true);
        var robot = new Robot();
        instantiate(robot);
        var mapleSim = SimulatedArena.class.getDeclaredField("instance");
        mapleSim.setAccessible(true);
        assertNull(mapleSim.get(null), "MapleSim was instantiated on the real robot.");
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