package robot.subsystems.shooter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.Convert;
import lib.Tunable;
import lib.commands.CmdSequence;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.ChargerSubsystem;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Shooter extends ChargerSubsystem {
    /** The yaw, pitch, and shooter speed for launched balls to reach the target. */
    public record Setpoint(Rotation2d yaw, double radPerSec, boolean isPossible) {
        /** Represents a Shooter setpoint with no data. */
        public static final Setpoint NULL = new Setpoint(Rotation2d.kZero, 0, false);
    }

    /** A location on the field. */
    public enum Target {
        HUB, GROUND
    }

    public enum IdleBehavior {
        SPINUP, COAST
    }

    public static final double FLYWHEEL_REDUCTION = 1.0;
    public static final Transform2d ROBOT_TO_LAUNCH_POINT =
        new Transform2d(Inches.of(6.07), Inches.zero(), Rotation2d.kZero);
    public static final Translation3d ROBOT_TO_SHOOTER_PIVOT_POINT =
        new Translation3d(Inches.of(9.06), Inches.zero(), Inches.of(11.776));
    public static final Distance FUEL_LAUNCH_HEIGHT = Inches.of(15.2 + 1.5);
    public static final Rotation2d LAUNCH_ANGLE = Rotation2d.fromDegrees(90 - 20);
    public static final ChassisSpeeds ZERO_VEL = new ChassisSpeeds();

    private final Tunable<Double>
        flywheelKp = Tunable.of(key("Flywheels/KP"), 2.3),
        flywheelKd = Tunable.of(key("Flywheels/KD"), 0.0),
        flywheelKv = Tunable.of(key("Flywheels/KV"), 0.02),
        flywheelTolerance = Tunable.of(key("Flywheels/Tolerance (Rad per s)"), 15.0),
        shotLimit = Tunable.of(key("Flywheels/ShootingCurrentLimit"), 60.0),
        spinupLimit = Tunable.of(key("Flywheels/SpinupCurrentLimit"), 20.0);
    private final Tunable<AngularVelocity>
        defaultSpinupVel = Tunable.of(key("Flywheels/DefaultSpinupVel"), RadiansPerSecond.of(200));
    private final KrakenFlywheels flywheelIO = new KrakenFlywheels();
    private final FlywheelDataAutoLogged flywheelInputs = new FlywheelDataAutoLogged();

    @AutoLogOutput(unit = "RadPerSec") private double flywheelErr = 0;

    /** Whether the flywheels and hood have reached their desired velocity and position, respectively. */
    public boolean atGoal(double toleranceMultiplier) {
        boolean result = flywheelErr < (flywheelTolerance.get() * toleranceMultiplier);
        Logger.recordOutput(key("AtGoal"), result);
        return result;
    }

    public Shooter() {
        configureGains();
        flywheelKp.onChange(this::configureGains);
        flywheelKd.onChange(this::configureGains);
        flywheelKv.onChange(this::configureGains);
        setCurrentLimit(shotLimit.get());
        var hoodPos = new Pose3d(
            ROBOT_TO_SHOOTER_PIVOT_POINT,
            new Rotation3d(0, 11 * Convert.DEGREES_TO_RADIANS, 0)
        );
        Logger.recordOutput("HoodPosition", hoodPos);
    }

    private void setCurrentLimit(double amps) {
        flywheelIO.setCurrentLimit(amps);
        Logger.recordOutput(key("CurrentLimit"), amps);
    }

    private void configureGains() {
        flywheelIO.setGains(flywheelKp.get(), flywheelKd.get(), flywheelKv.get());
    }

    // We use a separate method for the "impl" version of the setVelocityCmd
    // so that it isn't logged when spinupCmd() is scheduled.
    private Command setVelocityCmdImpl(Supplier<AngularVelocity> targetSupplier) {
        return this.run(() -> {
            var target = targetSupplier.get();
            flywheelIO.setVelocity(target);
            flywheelErr = Math.abs(flywheelInputs.velocity.minus(target).in(RadiansPerSecond));
        });
    }

    public Command setIdleBehaviorToCoastCmd() {
        var cmd = this.runOnce(() -> setDefaultCommand(coastCmd()))
            .ignoringDisable(true);
        return logged(cmd, "Change Idle Behavior to Coast");
    }

    public Command setIdleBehaviorToSpinupCmd() {
        var cmd = this.runOnce(() -> setDefaultCommand(spinupCmd()))
            .ignoringDisable(true);
        return logged(cmd, "Change Idle Behavior to Coast");
    }

    /** Run flywheels at coast mode with no output */
    public Command coastCmd() {
        return logged(this.run(flywheelIO::setCoast), "FlywheelCoast");
    }

    /** Spins up the shooter at the default angular velocity. */
    public Command spinupCmd() {
        return spinupCmd(defaultSpinupVel::get);
    }

    /**
     * Runs the flywheel at the designated angular velocity while
     * lowering the current limit, to allow it to build momentum.
     */
    public Command spinupCmd(Supplier<AngularVelocity> velocity) {
        var cmd = Commands.parallel(
            setVelocityCmdImpl(velocity),
            CmdSequence.of(
                Commands.runOnce(() -> setCurrentLimit(spinupLimit.get())),
                Commands.waitUntil(() -> atGoal(1.0)),
                Commands.runOnce(() -> setCurrentLimit(shotLimit.get()))
            )
        )
            .finallyDo(() -> setCurrentLimit(shotLimit.get()));
        return logged(cmd, "Spinup");
    }

    /** Runs the shooter at the commanded setpoint. */
    public Command setVelocityCmd(Supplier<AngularVelocity> velocity) {
        return logged(setVelocityCmdImpl(velocity), "SetVelocity");
    }

    public AngularVelocity velocity() {
        return flywheelInputs.velocity;
    }

    @Override
    public void loggedPeriodic() {
        flywheelIO.refreshData(flywheelInputs);
        Logger.processInputs("Shooter/Flywheels", flywheelInputs);
    }
}