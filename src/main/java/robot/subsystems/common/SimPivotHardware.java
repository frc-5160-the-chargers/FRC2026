package robot.subsystems.common;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import lib.hardware.MotorStats;
import robot.subsystems.pivot.PivotDataAutoLogged;

import static edu.wpi.first.units.Units.*;

public class SimPivotHardware extends PivotHardware {
    private final PivotSimConfig config;
    private final SingleJointedArmSim sim;
    private final PIDController pidController = new PIDController(0, 0, 0);

    public SimPivotHardware(PivotSimConfig config) {
        this.config = config;
        sim = new SingleJointedArmSim(
            config.motorKind(), config.reduction(),
            config.moi().in(KilogramSquareMeters),
            config.pivotLength().in(Meters),
            0, 2 * Math.PI,
            config.simulateGravity(), 0
        );
    }

    @Override
    public void setPDGains(double p, double d) {
        pidController.setPID(p, 0, d);
    }

    @Override
    public void refreshData(PivotDataAutoLogged data) {
        sim.update(0.02);
        data.radians = sim.getAngleRads();
        data.radiansPerSec = sim.getVelocityRadPerSec();
        data.motorStats = MotorStats.from(sim);
    }

    @Override
    public void setRadians(double targetRadians, double feedforwardV) {
        double volts = pidController.calculate(sim.getAngleRads(), targetRadians) + feedforwardV;
        setVolts(volts);
    }

    @Override
    public void setVolts(double volts) {
        // applies a current limit
        var rawVelocity = sim.getAngleRads() * config.reduction();
        double torque = config.motorKind().getTorque(config.currentLimit());
        double minVolts = config.motorKind().getVoltage(-torque, rawVelocity);
        double maxVolts = config.motorKind().getVoltage(torque, rawVelocity);
        sim.setInputVoltage(MathUtil.clamp(volts, minVolts, maxVolts));
    }

    @Override
    public void zeroEncoder() {
        sim.setState(0, 0);
    }
}