package robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;



import edu.wpi.first.wpilibj2.command.Command;
import robot.subsystems.ChargerSubsystem;

public class Climber extends ChargerSubsystem{
    private ClimberDataAutoLogged data = new ClimberDataAutoLogged();
    private SimClimberHardware io = new SimClimberHardware();

    public Climber() {
        io.setPDGains(ClimberConsts.KP.get(), ClimberConsts.KD.get());  // Assign PID Consts

        ClimberConsts.KP.onChange(() -> {
            io.setPDGains(ClimberConsts.KP.get(), ClimberConsts.KD.get());
            io.configureMotor();
        });

        ClimberConsts.KD.onChange(() -> {
            io.setPDGains(ClimberConsts.KP.get(), ClimberConsts.KD.get());
            io.configureMotor();
        });
    }

    @Override
    public void periodic() {
        io.refreshData(data);
        Logger.processInputs(getName(), data);
    }

    public Command setVoltage(double volts) {
        return this.run(() -> io.setVoltage(volts))
        .finallyDo(() -> io.setVoltage(0))
        .withName("climber set voltage");
    }

    public Command setPos(double radians) {
        return this.run(() -> io.setRadians(radians)).withName("setting climber radians " + radians);
    }

    public Command stop() {
        return this.run(() -> io.setVoltage(0.0)).withName("stop climber");
    }

    public Command extend() {
        return setPos(3);
    }

    public Command retract() {
        return setPos(0);
    }
}
