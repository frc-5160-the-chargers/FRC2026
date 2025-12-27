package robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lib.commands.CmdLogger;
import lib.hardware.SignalBatchRefresher;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import robot.subsystems.drive.AiRobotTunerConstants;
import robot.subsystems.drive.SwerveConfig;
import robot.subsystems.drive.SwerveDrive;

import static lib.commands.NonBlockingCmds.sequence;

public class DefenseSimulator extends LoggedRobot {
    private class CustomArena extends Arena2025Reefscape {
        public boolean robotsAreColliding() {
            try {
                return physicsWorld.isInContact(drive.getMapleSim(), secondaryDrive.getMapleSim());
            } catch (Exception e) {
                return false;
            }
        }
    }

    private final CustomArena arena = new CustomArena();
    private final SwerveDrive drive, secondaryDrive;

    private final Pose2d
        startPoint = new Pose2d(3, 7, Rotation2d.kZero),
        endPoint = new Pose2d(13, 7, Rotation2d.kZero);

    private Pose2d goal = startPoint;

    public DefenseSimulator() {
        SimulatedArena.overrideInstance(arena);
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();

        drive = new SwerveDrive(SwerveConfig.DEFAULT);
        drive.setDefaultCommand(drive.driveCmd(new DriverController(0)::getSwerveRequest));
        drive.resetPose(new Pose2d(5, 7, Rotation2d.kZero));
        drive.setSimulatePoseEstDrift(false);

        secondaryDrive = new SwerveDrive(
            new SwerveConfig(
                AiRobotTunerConstants.DrivetrainConstants,
                AiRobotTunerConstants.FrontLeft, AiRobotTunerConstants.FrontRight,
                AiRobotTunerConstants.BackLeft, AiRobotTunerConstants.BackRight
            )
        );
        secondaryDrive.setSimulatePoseEstDrift(false);
        secondaryDrive.setName("MockSecondaryDrivetrain");
        secondaryDrive.setDefaultCommand(
            sequence(
                secondaryDrive.pathfindCmd(() -> goal),
                Commands.runOnce(() -> goal = goal.equals(startPoint) ? endPoint : startPoint)
            )
                .repeatedly()
                .withName("DefaultSecondaryDrive")
        );

        var avoidanceCmd = secondaryDrive.driveCmd(() -> {
            var req = new SwerveRequest.FieldCentric();
            req.RotationalRate = 7 * (goal.equals(startPoint) ? -1 : 1);
            var delta = drive.getPose().getTranslation().minus(secondaryDrive.getPose().getTranslation());
            double invert = (secondaryDrive.getPose().getX() > 10 ? 1 : -1);
            if (Math.abs(delta.getX()) - Math.abs(delta.getY()) > 0.2) {
                req.VelocityY = 4.4 * invert;
            } else {
                Logger.recordOutput("IsOrthogonal", true);
                req.VelocityX = 4.4 * -invert;
            }
            return req;
        })
            .until(() -> Math.abs(secondaryDrive.getPose().getY() - drive.getPose().getY()) > 1.0)
            .withName("CollisionAvoidance");
        new Trigger(arena::robotsAreColliding)
            .onTrue(
                Commands.waitSeconds(1).andThen(new ScheduleCommand(avoidanceCmd))
            );
        secondaryDrive.resetPose(new Pose2d(3, 7, Rotation2d.kZero));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SignalBatchRefresher.refreshAll();
        CmdLogger.periodic(false);
//        if (opp != null) secondaryDrive.getRepulsor().removeObstacle(opp);
//        opp = new RepulsorFieldPlanner.TeardropObstacle(
//            drive.getPose().getTranslation(),
//            0.4, 1.6, .7, 0.8, 1.5
//        );
//        secondaryDrive.getRepulsor().addObstacle(opp);
    }
}
