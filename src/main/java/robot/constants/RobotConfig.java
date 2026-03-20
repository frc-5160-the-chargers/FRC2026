package robot.constants;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import lib.AllianceColor;
import lib.RobotMode;
import lib.Tunable;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import robot.subsystems.drive.SwerveConfig;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.drive.TunerConstants;

import java.io.File;
import java.util.Optional;

import static choreo.util.ChoreoAllianceFlipUtil.flip;
import static edu.wpi.first.units.Units.Inches;

@SuppressWarnings("DataFlowIssue")
public class RobotConfig {
    private static final Tunable<Double> BUMP_ANGLE_RAD = Tunable.of("BumpTravelAngleRad", -2.12);

    /** The configuration for this year's swerve robot. */
    public static final SwerveConfig swerveCfg = new SwerveConfig(
        "Swerve",
        Inches.of(27 + 2 * 3.5),
        Inches.of(26 + 2 * 3.5),
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft, TunerConstants.FrontRight,
        TunerConstants.BackLeft, TunerConstants.BackRight
    );

    /** The canivore we use on the robot. */
    public static final CANBus CANIVORE = TunerConstants.kCANBus;

    /**
     * A logger that will only publish to networktables.
     * Useful if data is desired to be logged to a dashboard.
     */
    public static final EpilogueBackend dashboardLogger =
        new NTEpilogueBackend(NetworkTableInstance.getDefault());

    /** Creates an intake simulation characterizing this year's OTB intake. */
    public static Optional<IntakeSimulation> createIntakeSim(SwerveSubsystem drive) {
        if (!RobotMode.isSim()) return Optional.empty();
        var sim = IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            drive.getSim(),
            Inches.of(27),
            Inches.of(7.5),
            IntakeSimulation.IntakeSide.BACK,
            30
        );
        return Optional.of(sim);
    }

    /** Gets the angle needed to travel over the bump. */
    public static Optional<Rotation2d> getBumpTravelingAngle() {
        var angle = Rotation2d.fromRadians(BUMP_ANGLE_RAD.get());
        return Optional.of(AllianceColor.isRed() ? flip(angle) : angle);
    }

    /** Logs a choreo trajectory. */
    public static void logTrajectory(Trajectory<SwerveSample> rawTraj, boolean isStart) {
        Logger.recordOutput("CurrentTraj/Name", rawTraj.name());
        if (RobotMode.get() != RobotMode.REPLAY && DriverStation.isFMSAttached()) {
            return; // don't log trajectory during matches, use replay mode to do so instead
        }
        var samples = rawTraj.samples().toArray(new SwerveSample[0]);
        Logger.recordOutput("CurrentTraj/Samples", samples);
    }

    /** Logging config for the main robot. */
    public static void initLoggingForMainBot() {
        if (System.getenv("test") != null) {
            return; // Unit test, don't need logging here
        }
        if (RobotMode.get() == RobotMode.REPLAY) {
            // Checks for an opened log file in AdvantageScope.
            String path = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(path));
            Logger.addDataReceiver(
                new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_replay"))
            );
        } else {
            var ntLogger = new NT4Publisher();
            Logger.addDataReceiver(data -> {
                if (DriverStation.isFMSAttached()) return;
                ntLogger.putTable(data);
            });
            boolean logToUsbDrive = RobotMode.isSim() || new File("/U/logs/").exists();
            Logger.addDataReceiver(
                logToUsbDrive ? new WPILOGWriter() : new WPILOGWriter("/home/lvuser/logs")
            );
            Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
            Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
            Logger.recordMetadata("Timestamp", BuildConstants.BUILD_DATE);
            Logger.recordMetadata("GitDirty", switch (BuildConstants.DIRTY) {
                case 0 -> "All changes commited";
                case 1 -> "There are uncommited changes; replay might be inaccurate";
                default -> "Unknown";
            });
            Logger.recordMetadata("LoggedToUSBDrive", logToUsbDrive ? "Yes" : "No");
            StatusLogger.disableAutoLogging();
            SignalLogger.enableAutoLogging(false);
        }
        Logger.start();
    }
}
