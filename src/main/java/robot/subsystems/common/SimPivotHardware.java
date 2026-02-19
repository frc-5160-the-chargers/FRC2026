package robot.subsystems.common;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import lib.hardware.MotorStats;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;

public class SimPivotHardware extends PivotHardware {
    private final PivotSimConfig config;
    private final SingleJointedArmSim sim;
    private final PIDController pidController = new PIDController(0, 0, 0);
    private double currentLimit = 300.0;

    public SimPivotHardware(PivotSimConfig config) {
        this.config = config;
        sim = new SingleJointedArmSim(
            config.motorKind(), config.reduction(),
            config.moi().in(KilogramSquareMeters),
            config.pivotLength().in(Meters),
            -2 * Math.PI, 2 * Math.PI,
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
        sim.setInputVoltage(volts);
    }

    @Override
    public void setCurrentLimit(double amps) {
        this.currentLimit = amps;
    }

    @Override
    public void zeroEncoder() {
        sim.setState(0, 0);
    }
}