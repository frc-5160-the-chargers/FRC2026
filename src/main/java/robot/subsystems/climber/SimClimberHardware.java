package robot.subsystems.climber;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;


public class SimClimberHardware extends ClimberHardware {
    private final DCMotorSim motor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(2), 0.01, 7),
        DCMotor.getNeoVortex(2)
    );

    @Override
    public void refreshData(ClimberData data) {
        motor.update(0.02);
        data.volts = motor.getInputVoltage();
        data.pos = motor.getAngularPositionRad();

        if (DriverStation.isDisabled()) {
            motor.setInputVoltage(0);
        }
        
    }

    @Override
    public void setVoltage(double volts) {
        motor.setInputVoltage(volts);
    }
}