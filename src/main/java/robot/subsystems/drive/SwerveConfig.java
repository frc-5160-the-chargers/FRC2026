package robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import robot.constants.ChoreoVars;

import static edu.wpi.first.units.Units.*;

/**
 * Represents a selection of tuner constants for the drivetrain.
 * @param moduleConsts Must be in front left, front right, back left, and back right order.
 */
public record SwerveConfig(
    String name,
    Distance bumperThickness,
    SwerveDrivetrainConstants driveConsts,
    SwerveModuleConstants<?, ?, ?>... moduleConsts
) {
    public SwerveConfig(
        SwerveDrivetrainConstants driveConsts,
        SwerveModuleConstants<?, ?, ?>... moduleConsts
    ) {
        this("SwerveSubsystem", Inches.of(3.5), driveConsts, moduleConsts);
    }

    /**
     * Gets the position of every swerve module in the drivetrain,
     * as an array of {@link Translation2d} objects.
     */
    public Translation2d[] moduleTranslations() {
        var data = new Translation2d[4];
        for (int i = 0; i < 4; i++) {
            data[i] = new Translation2d(moduleConsts[i].LocationX, moduleConsts[i].LocationY);
        }
        return data;
    }

    /** Gets this drivetrain's config for maplesim. */
    public DriveTrainSimulationConfig mapleSimConfig() {
        var moduleConfigs = new SwerveModuleSimulationConfig[4];
        for (int i = 0; i < 4; i++) {
            moduleConfigs[i] = new SwerveModuleSimulationConfig(
                DCMotor.getKrakenX60(1),
                DCMotor.getKrakenX44(1),
                moduleConsts[i].DriveMotorGearRatio,
                moduleConsts[i].SteerMotorGearRatio,
                Volts.of(moduleConsts[i].DriveFrictionVoltage),
                Volts.of(moduleConsts[i].SteerFrictionVoltage),
                Meters.of(moduleConsts[i].WheelRadius),
                KilogramSquareMeters.of(moduleConsts[i].SteerInertia),
                ChoreoVars.cof
            );
        }
        return DriveTrainSimulationConfig.Default()
            .withRobotMass(ChoreoVars.robotMass)
            .withCustomModuleTranslations(moduleTranslations())
            .withGyro(COTS.ofPigeon2())
            .withSwerveModules(moduleConfigs)
            .withBumperSize(
                Meters.of(2 * (moduleConsts[0].LocationX + bumperThickness.in(Meters))),
                Meters.of(2 * (moduleConsts[0].LocationY + bumperThickness.in(Meters)))
            );
    }

    /** The distance from the center of the robot to one of the modules. */
    public Distance drivebaseRadius() {
        return Meters.of(Math.hypot(moduleConsts[0].LocationX, moduleConsts[0].LocationY));
    }

    /** The max linear velocity of the robot. */
    public LinearVelocity maxVel() {
        return MetersPerSecond.of(moduleConsts[0].SpeedAt12Volts);
    }

    /** The max angular velocity of the robot. */
    public AngularVelocity maxAngularVel() {
        return RadiansPerSecond.of(moduleConsts[0].SpeedAt12Volts / drivebaseRadius().in(Meters));
    }
}
