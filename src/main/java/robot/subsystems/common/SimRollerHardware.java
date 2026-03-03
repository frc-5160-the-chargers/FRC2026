package robot.subsystems.common;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import lib.hardware.MotorStats;

public class SimRollerHardware extends RollerHardware {
    private final DCMotorSim sim;

    public SimRollerHardware(DCMotor motorKind, double gearing) {
        var system = LinearSystemId.createDCMotorSystem(motorKind, 0.001, gearing);
        sim = new DCMotorSim(system, motorKind);
    }

    @Override
    public void refreshData(RollerDataAutoLogged data) {
        sim.update(0.02);
        data.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
        data.motorStats = new MotorStats[] {MotorStats.from(sim)};
        if (DriverStation.isDisabled()) setVolts(0);
    }

    @Override
    public void setVolts(double volts) {
        sim.setInputVoltage(volts);
    }
}
