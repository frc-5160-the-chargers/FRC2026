package robot.subsystems.shooter.flywheels;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimFlywheelHardware extends FlywheelHardware {
    private final DCMotorSim motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(2), 0.01, 2),
            DCMotor.getKrakenX60Foc(2)
    );
    private final PIDController pid = new PIDController(0,0,0);

    @Override
    public void setVoltage(double volts) {
        motor.setInputVoltage(volts);
    }
}
