// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import robot.constants.RobotConfig;


public class HubShiftUtil {
    public enum AllianceOverride {
        NONE, OPPOSING_ALLIANCE, YOUR_ALLIANCE
    }

    public enum ShiftEnum {
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME,
        AUTO,
        DISABLED
    }

    public record ShiftInfo(
        ShiftEnum currentShift,
        double elapsedTime,
        double remainingTime,
        boolean active
    ) {}

    private static final Timer shiftTimer = new Timer();
    private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

    private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
    private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

    private static final double
        minFuelCountDelay = 1.0,
        maxFuelCountDelay = 2.0,
        shiftEndFuelCountExtension = 3.0,
        minTimeOfFlight = 0.85,
        maxTimeOfFlight = 1.5,
        approachingActiveFudge = -1 * (minTimeOfFlight + minFuelCountDelay),
        endingActiveFudge =
            shiftEndFuelCountExtension + -1 * (maxTimeOfFlight + maxFuelCountDelay);

    public static final double autoEndTime = 20.0, teleopDuration = 140.0;
    private static final boolean[] activeSchedule = {true, true, false, true, false, true};
    private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};

    private static final LoggedDashboardChooser<AllianceOverride> allianceOverride =
        new LoggedDashboardChooser<>("AllianceOverride");

    static {
        allianceOverride.addDefaultOption("None", AllianceOverride.NONE);
        allianceOverride.addOption("Opposing Alliance Outscores in Auto", AllianceOverride.OPPOSING_ALLIANCE);
        allianceOverride.addOption("Your Alliance Outscore in Auto", AllianceOverride.YOUR_ALLIANCE);
    }

    public static Alliance getFirstActiveAlliance() {
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        // Return override value
        if (!DriverStation.isFMSAttached() && allianceOverride.get() != AllianceOverride.NONE) {
            return allianceOverride.get() == AllianceOverride.OPPOSING_ALLIANCE
                ? (alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue)
                : (alliance == Alliance.Blue ? Alliance.Blue : Alliance.Red);
        }

        // Return FMS value
        String message = DriverStation.getGameSpecificMessage();
        if (!message.isEmpty()) {
            char character = message.charAt(0);
            if (character == 'R') {
                return Alliance.Blue;
            } else if (character == 'B') {
                return Alliance.Red;
            }
        }

        // Return default value
        return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
    }

    /** Starts the timer at the begining of teleop. */
    public static void initialize() {
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(shiftTimer::restart));
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(shiftTimer::restart));
        RobotModeTriggers.disabled()
            .onTrue(
                Commands.runOnce(shiftTimer::restart).ignoringDisable(true)
            );
    }

    public static void logData() {
        var dashboard = RobotConfig.dashboardLogger.getNested("SmartDashboard");
        // Publish match time
        dashboard.log("Match Time", DriverStation.getMatchTime());

        // Update from HubShiftUtil
        dashboard.log(
            "Remaining Shift Time",
            String.format("%.1f", Math.max(HubShiftUtil.getShiftedShiftInfo().remainingTime(), 0.0))
        );
        dashboard.log("Shift Active", HubShiftUtil.getShiftedShiftInfo().active());
        dashboard.log("Game State", HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
        var alliance = HubShiftUtil.getFirstActiveAlliance();
        dashboard.log("FirstActiveAlliance", alliance);
        Logger.recordOutput("FirstActiveAlliance", alliance);
    }

    private static boolean[] getSchedule() {
        boolean[] currentSchedule;
        Alliance startAlliance = getFirstActiveAlliance();
        currentSchedule =
            startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
                ? activeSchedule
                : inactiveSchedule;
        return currentSchedule;
    }

    private static ShiftInfo getShiftInfo(
        boolean[] currentSchedule,
        double[] shiftStartTimes,
        double[] shiftEndTimes
    ) {
        double currentTime = shiftTimer.get();
        double stateTimeElapsed = shiftTimer.get();
        double stateTimeRemaining = 0.0;
        boolean active = false;
        ShiftEnum currentShift = ShiftEnum.DISABLED;

        if (DriverStation.isAutonomousEnabled()) {
            stateTimeElapsed = currentTime;
            stateTimeRemaining = autoEndTime - currentTime;
            active = true;
            currentShift = ShiftEnum.AUTO;
        } else if (DriverStation.isEnabled()) {
            int currentShiftIndex = -1;
            for (int i = 0; i < shiftStartTimes.length; i++) {
                if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
                    currentShiftIndex = i;
                    break;
                }
            }
            if (currentShiftIndex < 0) {
                // After last shift, so assume endgame
                currentShiftIndex = shiftStartTimes.length - 1;
            }

            // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
            stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
            stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

            // If the state is the same as the last shift, combine the elapsed time
            if (currentShiftIndex > 0) {
                if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
                    stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
                }
            }

            // If the state is the same as the next shift, combine the remaining time
            if (currentShiftIndex < shiftEndTimes.length - 1) {
                if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
                    stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
                }
            }

            active = currentSchedule[currentShiftIndex];
            currentShift = shiftsEnums[currentShiftIndex];
        }
        return new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
    }

    public static ShiftInfo getOfficialShiftInfo() {
        return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
    }

    public static ShiftInfo getShiftedShiftInfo() {
        boolean[] shiftSchedule = getSchedule();
        // Starting active
        if (shiftSchedule[1]) {
            double[] shiftedShiftStartTimes = {
                0.0,
                10.0,
                35.0 + endingActiveFudge,
                60.0 + approachingActiveFudge,
                85.0 + endingActiveFudge,
                110.0 + approachingActiveFudge
            };
            double[] shiftedShiftEndTimes = {
                10.0,
                35.0 + endingActiveFudge,
                60.0 + approachingActiveFudge,
                85.0 + endingActiveFudge,
                110.0 + approachingActiveFudge,
                140.0
            };
            return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
        }
        double[] shiftedShiftStartTimes = {
            0.0,
            10.0 + endingActiveFudge,
            35.0 + approachingActiveFudge,
            60.0 + endingActiveFudge,
            85.0 + approachingActiveFudge,
            110.0
        };
        double[] shiftedShiftEndTimes = {
            10.0 + endingActiveFudge,
            35.0 + approachingActiveFudge,
            60.0 + endingActiveFudge,
            85.0 + approachingActiveFudge,
            110.0,
            140.0
        };
        return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
    }
}
