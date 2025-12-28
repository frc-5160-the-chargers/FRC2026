package robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import robot.constants.ChoreoVars;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

/**
 * Represents a selection of tuner constants for the drivetrain.
 * @param encoderStdDevs Confidence of the drivetrain's encoder measurements
 *                       in the x, y, and theta directions. Usually just 3 0.1s.
 * @param moduleConsts Must be in front left, front right, back left, and back right order.
 */
public record SwerveConfig(
    Vector<N3> encoderStdDevs,
    Distance bumperThickness,
    SwerveDrivetrainConstants driveConsts,
    SwerveModuleConstants<?, ?, ?>... moduleConsts
) {
    public SwerveConfig(
        SwerveDrivetrainConstants driveConsts,
        SwerveModuleConstants<?, ?, ?>... moduleConsts
    ) {
        this(VecBuilder.fill(0.1, 0.1, 0.1), Inches.of(3.5), driveConsts, moduleConsts);
    }

    /** The configuration for the main drivetrain. */
    public static final SwerveConfig DEFAULT = new SwerveConfig(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft, TunerConstants.FrontRight,
        TunerConstants.BackLeft, TunerConstants.BackRight
    );

    /**
     * Gets the position of every swerve module in the drivetrain,
     * as an array of {@link Translation2d} objects.
     */
    public Translation2d[] moduleTranslations() {
        var data = new Translation2d[moduleConsts.length];
        for (int i = 0; i < 4; i++) {
            data[i] = new Translation2d(moduleConsts[i].LocationX, moduleConsts[i].LocationY);
        }
        return data;
    }

    /** Gets this drivetrain's config for maplesim. */
    public DriveTrainSimulationConfig mapleSimConfig() {
        var moduleCfg = moduleConsts[0];
        var mapleSimModuleCfg = new SwerveModuleSimulationConfig(
            DCMotor.getKrakenX60(1),
            DCMotor.getKrakenX44(1),
            moduleCfg.DriveMotorGearRatio,
            moduleCfg.SteerMotorGearRatio,
            Volts.of(moduleCfg.DriveFrictionVoltage),
            Volts.of(moduleCfg.SteerFrictionVoltage),
            Meters.of(moduleCfg.WheelRadius),
            KilogramSquareMeters.of(moduleCfg.SteerInertia),
            ChoreoVars.cof
        );
        return DriveTrainSimulationConfig.Default()
            .withRobotMass(ChoreoVars.botMass)
            .withCustomModuleTranslations(moduleTranslations())
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(mapleSimModuleCfg)
            .withBumperSize(
                Meters.of(2 * (moduleCfg.LocationX + bumperThickness.in(Meters))),
                Meters.of(2 * (moduleCfg.LocationY + bumperThickness.in(Meters)))
            );
    }

    /** The distance from the center of the robot to one of the modules. */
    public Distance drivebaseRadius() {
        return Meters.of(Math.hypot(moduleConsts[0].LocationX, moduleConsts[0].LocationY));
    }
}
