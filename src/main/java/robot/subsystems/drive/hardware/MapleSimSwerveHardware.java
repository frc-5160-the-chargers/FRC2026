package robot.subsystems.drive.hardware;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import robot.SharedData;
import robot.subsystems.drive.SwerveConfig;

import static edu.wpi.first.units.Units.Seconds;

/** Swerve hardware that uses MapleSim, a library that simulates on-field collisions. */
public class MapleSimSwerveHardware extends SwerveHardware {
    private static final Pose2d INITIAL_POSE = new Pose2d(7, 2, Rotation2d.kZero);
    private static final double SIM_UPDATE_PERIOD = 0.002;
    private static final Notifier DATA_UPDATER =
        new Notifier(() -> SimulatedArena.getInstance().simulationPeriodic());

    private final Pigeon2SimState gyroSim;
    private final SwerveDriveSimulation mapleSim;

    private MapleSimSwerveHardware(SwerveConfig config) {
        super(config);
        this.mapleSim = new SwerveDriveSimulation(config.mapleSimConfig(), INITIAL_POSE);
        this.gyroSim = super.drivetrain.getPigeon2().getSimState();
        super.drivetrain.resetTranslation(INITIAL_POSE.getTranslation());
        // these values simulate drag.
        mapleSim.setLinearDamping(0.7);
        mapleSim.setAngularDamping(0.7);
        SimulatedArena.getInstance().addDriveTrainSimulation(mapleSim);
        SimulatedArena.overrideSimulationTimings(Seconds.of(SIM_UPDATE_PERIOD), 1);
        DATA_UPDATER.startPeriodic(SIM_UPDATE_PERIOD);
        for (int i = 0; i < 4; i++) {
            var module = super.drivetrain.getModule(i);
            var sim = mapleSim.getModules()[i];
            module.getSteerMotor().getSimState().setMotorType(MotorType.KrakenX44);
            sim.useDriveMotorController(new TalonFXSim(module.getDriveMotor(), null));
            sim.useSteerMotorController(new TalonFXSim(module.getSteerMotor(), module.getEncoder()));
        }
    }

    /** Creates an instance of a {@link MapleSimSwerveHardware}. */
    public static MapleSimSwerveHardware create(SwerveConfig config) {
        // offsets that aren't applicable to sim must be cleared before the swerve hardware is created.
        for (var moduleConfig: config.moduleConsts()) {
            moduleConfig.EncoderOffset = 0;
            moduleConfig.DriveMotorInverted = false;
            moduleConfig.SteerMotorInverted = false;
            moduleConfig.EncoderInverted = false;
            moduleConfig.CouplingGearRatio = 0;
        }
        return new MapleSimSwerveHardware(config);
    }

    @Override
    public void refreshData(SwerveDataAutoLogged data) {
        SharedData.truePoseInSim = mapleSim.getSimulatedDriveTrainPose();
        gyroSim.setRawYaw(SharedData.truePoseInSim.getRotation().getMeasure());
        Logger.recordOutput(super.name + "TruePose", SharedData.truePoseInSim);
        super.refreshData(data);
    }

    @Override
    public void resetNotReplayedPose(Pose2d pose) {
        mapleSim.setSimulationWorldPose(pose);
        super.drivetrain.resetTranslation(pose.getTranslation());
    }

    /**
     * MapleSim will call the updateControlSignal() method, allowing us to override
     * the position & velocity values read from a TalonFX with simulated values.
     */
    private record TalonFXSim(TalonFX motor, @Nullable CANcoder encoder) implements SimulatedMotorController {
        @Override
        public Voltage updateControlSignal(
            Angle mechanismAngle, // position w/ gearing
            AngularVelocity mechanismVelocity,
            Angle encoderAngle, // position without gearing
            AngularVelocity encoderVelocity
        ) {
            if (encoder != null) {
                encoder.getSimState().setRawPosition(mechanismAngle);
                encoder.getSimState().setVelocity(mechanismVelocity);
            }
            motor.getSimState().setRawRotorPosition(encoderAngle);
            motor.getSimState().setRotorVelocity(encoderVelocity);
            motor.getSimState().setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            return motor.getSimState().getMotorVoltageMeasure();
        }
    }
}