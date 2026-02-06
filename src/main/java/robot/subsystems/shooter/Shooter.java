package robot.subsystems.shooter;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import lib.RobotMode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import robot.subsystems.ChargerSubsystem;
import robot.subsystems.common.PivotDataAutoLogged;
import robot.subsystems.common.PivotHardware;
import robot.subsystems.shooter.ShooterConsts.ShooterSetpoint;
import robot.subsystems.shooter.flywheels.FlywheelDataAutoLogged;
import robot.subsystems.shooter.flywheels.FlywheelHardware;
import robot.subsystems.shooter.flywheels.KrakenFlywheelHardware;
import robot.subsystems.shooter.flywheels.SimFlywheelHardware;

import static robot.subsystems.shooter.ShooterConsts.HOOD_MOTION_CONSTRAINTS;

public class Shooter extends ChargerSubsystem {
    private final FlywheelHardware flywheelIO = switch (RobotMode.get()) {
        case REAL -> new KrakenFlywheelHardware();
        case SIM -> new SimFlywheelHardware();
        case REPLAY -> new FlywheelHardware();
    };
    private final PivotHardware hoodIO = switch (RobotMode.get()) {
        case REAL -> new NeoShooterHood();
        case SIM, REPLAY -> new PivotHardware();
    };
    private final FlywheelDataAutoLogged flywheelInputs = new FlywheelDataAutoLogged();
    private final PivotDataAutoLogged hoodInputs = new PivotDataAutoLogged();

    private final TrapezoidProfile hoodProfile = new TrapezoidProfile(HOOD_MOTION_CONSTRAINTS);
    @AutoLogOutput private TrapezoidProfile.State hoodSetpoint = new TrapezoidProfile.State();

    /** The current flywheel velocity. */
    public AngularVelocity getVelocity() {
        return flywheelInputs.velocity;
    }

    /** Modifies the shooter's setpoint. */
    public void setTarget(ShooterSetpoint target) {
        if (!target.valid()) return;
        flywheelIO.setVelocity(target.targetVelocity());
        var hoodGoal = new TrapezoidProfile.State(target.pitch().getRadians(), 0);
        hoodSetpoint = hoodProfile.calculate(0.02, hoodSetpoint, hoodGoal);
        hoodIO.setRadians(hoodSetpoint.position, hoodSetpoint.velocity);
    }

    @Override
    public void loggedPeriodic() {
        flywheelIO.refreshData(flywheelInputs);
        hoodIO.refreshData(hoodInputs);
        Logger.processInputs("Shooter/Flywheels", flywheelInputs);
        Logger.processInputs("Shooter/Hood", hoodInputs);
    }
}
