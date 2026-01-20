package robot.subsystems.common;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import lib.hardware.MotorStats;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

public class SimRollerHardware extends RollerHardware {
    private final DCMotorSim sim;

    public SimRollerHardware(DCMotor motorKind, MomentOfInertia moi, double gearing) {
        var system = LinearSystemId.createDCMotorSystem(motorKind, moi.in(KilogramSquareMeters), gearing);
        sim = new DCMotorSim(system, motorKind);
    }

    @Override
    public void refreshData(RollerDataAutoLogged data) {
        data.radiansPerSec = sim.getAngularVelocityRadPerSec();
        data.motorStats[0] = MotorStats.from(sim);
    }

    @Override
    public void setVolts(double volts) {
        sim.setInputVoltage(volts);
    }
}
