package robot.subsystems.intakepivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import lib.hardware.MotorStats;

import static edu.wpi.first.units.Units.*;
import static robot.subsystems.intakepivot.PivotConsts.*;

public class SimPivotHardware extends PivotHardware {
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
        MOTOR_KIND, REDUCTION, MOI_KG_METERS_SQ,
        PIVOT_LENGTH.in(Meters), 0, 2 * Math.PI,
        true, 0
    );
    private final PIDController pidController = new PIDController(0, 0, 0);

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
        var rawVelocity = sim.getAngleRads() * REDUCTION;
        double torque = MOTOR_KIND.getTorque(CURRENT_LIMIT_AMPS);
        double minVolts = MOTOR_KIND.getVoltage(-torque, rawVelocity);
        double maxVolts = MOTOR_KIND.getVoltage(torque, rawVelocity);
        sim.setInputVoltage(MathUtil.clamp(volts, minVolts, maxVolts));
    }

    @Override
    public void zeroEncoder() {
        sim.setState(0, 0);
    }
}