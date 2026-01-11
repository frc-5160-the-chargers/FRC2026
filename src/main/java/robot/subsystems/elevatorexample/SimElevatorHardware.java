package robot.subsystems.elevatorexample;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import lib.hardware.MotorStats;

import static edu.wpi.first.units.Units.*;
import static robot.subsystems.elevatorexample.ElevatorConsts.*;

public class SimElevatorHardware extends ElevatorHardware {
    private final ElevatorSim sim = new ElevatorSim(
        MOTOR_KIND, REDUCTION, CARRIAGE_MASS.in(Kilograms),
        RADIUS.in(Meters), MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters),
        true,
        0
    );
    private final PIDController pidController = new PIDController(0, 0, 0);
    private double currentRadians = 0;

    @Override
    public void setPDGains(double p, double d) {
        pidController.setPID(p, 0, d);
    }

    @Override
    public void refreshData(ElevatorDataAutoLogged data) {
        sim.update(0.02);
        currentRadians = sim.getPositionMeters() / RADIUS.in(Meters);
        data.radians = currentRadians;
        data.radiansPerSec = sim.getVelocityMetersPerSecond() / RADIUS.in(Meters);
        data.leaderStats = MotorStats.from(sim);
    }

    @Override
    public void setRadians(double targetRadians, double feedforwardV) {
        setVolts(
            pidController.calculate(currentRadians, targetRadians) + feedforwardV
        );
    }

    @Override
    public void setVolts(double volts) {
        // applies a current limit
        var rawVelocity = currentRadians * REDUCTION;
        double torque = MOTOR_KIND.getTorque(CURRENT_LIMIT);
        double minVolts = MOTOR_KIND.getVoltage(-torque, rawVelocity);
        double maxVolts = MOTOR_KIND.getVoltage(torque, rawVelocity);
        sim.setInputVoltage(MathUtil.clamp(volts, minVolts, maxVolts));
    }

    @Override
    public void zeroEncoder() {
        sim.setState(0, 0);
    }
}