package robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lib.hardware.SignalBatchRefresher;
import robot.subsystems.drive.AiRobotTunerConstants;
import robot.subsystems.drive.SwerveConfig;
import robot.subsystems.drive.SwerveDrive;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

@SuppressWarnings("ALL")
public class OffenseSimulator extends LoggedRobot {
    private final DriverController
        controller1 = new DriverController(0),
        controller2 = new DriverController(1);

    private final SwerveDrive drive, secondaryDrive;
    private final Timer holdTimer = new Timer();
    private boolean isOnCooldown = false;

    private Translation2d getDistBetweenDrivetrains() {
        var driveTrans = drive.truePose().getTranslation();
        var delta = driveTrans.minus(secondaryDrive.truePose().getTranslation());
        Logger.recordOutput("DeltaDist", delta.getNorm());
        return delta;
    }

    public OffenseSimulator() {
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();

        drive = new SwerveDrive(SwerveConfig.DEFAULT);
        drive.setDefaultCommand(drive.driveCmd(controller1::getSwerveRequest));
        drive.resetPose(new Pose2d(5, 7, Rotation2d.kZero));

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
            secondaryDrive
                .pathfindCmd(() -> new Pose2d(6.8, drive.truePose().getY(), Rotation2d.kZero))
                .repeatedly()
        );
        new Trigger(() -> getDistBetweenDrivetrains().getNorm() < 1.3)
            .and(() -> drive.truePose().getTranslation().getDistance(new Translation2d(4.5, 4)) > 1.5)
            .whileTrue(
                secondaryDrive.pathfindCmd(() -> {
                    var dist = getDistBetweenDrivetrains();
                    return new Pose2d(
                        drive.truePose().getTranslation().plus(
                            new Translation2d(0.5, dist.getAngle())
                        ),
                        Rotation2d.kZero
                    );
                })
                    .withTimeout(3)
                    .andThen(
                        secondaryDrive.driveCmd(() -> new SwerveRequest.Idle())
                            .withTimeout(4)
                    )
            );
        secondaryDrive.resetPose(new Pose2d(6.8, 7, Rotation2d.k180deg));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SignalBatchRefresher.refreshAll();
    }
}
