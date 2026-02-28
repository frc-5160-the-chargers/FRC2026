package robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.commands.NonBlockingCmds;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import robot.controllers.DriverController;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.intake.GroundIntake;
import robot.subsystems.serializer.Serializer;
import robot.subsystems.shooter.DataTypes.ShooterSetpoint;
import robot.subsystems.shooter.Shooter;
import robot.subsystems.shooter.math.HubShotCalcsKt;

import java.util.ArrayList;
import java.util.Optional;

import static choreo.util.ChoreoAllianceFlipUtil.flip;
import static lib.commands.CmdLogger.logged;

@SuppressWarnings("unused")
@RequiredArgsConstructor
public class Superstructure {
    // Constants
    private static final ArrayList<Rectangle2d> hubNoShootZones = new ArrayList<>();
    static {
        var blueZone = new Rectangle2d(new Translation2d(3.8, 8), new Translation2d(5.3, 0));
        var redZone = new Rectangle2d(flip(blueZone.getCenter()), blueZone.getXWidth(), blueZone.getYWidth());
        hubNoShootZones.add(blueZone);
        hubNoShootZones.add(redZone);
    }

    // Constructor Parameters
    private final DriverController controller;
    private final SwerveSubsystem drive;
    private final GroundIntake groundIntake;
    private final Shooter shooter;
    private final Serializer serializer;

    // State & util methods
    @AutoLogOutput private ShooterSetpoint shotSetpoint = ShooterSetpoint.NULL;
    @Getter private Optional<Rotation2d> rotationOverride = Optional.empty();

    // Commands
    public Command shootInHubCmd() {
        var speedDebouncer = new Debouncer(0.2);
        var waitForShot = Commands.waitUntil(() -> {
            if (!shotSetpoint.flywheelsAreReady()) return false;
            if (!shooter.hoodAtGoal) return false;
            for (var zone: hubNoShootZones) {
                if (zone.contains(drive.getPose().getTranslation())) return false;
            }
            return true;
        });
        var runSerializer = NonBlockingCmds.sequence(
            waitForShot, Commands.waitSeconds(0.5), serializer.runCmd()
        );
        var aimAndShoot = shooter.runCmd(inputs -> {
            shotSetpoint = HubShotCalcsKt.calcHubShotSetpoint(
                drive.getSim().getSimulatedDriveTrainPose(), drive.getFieldSpeeds(),
                inputs, speedDebouncer, shotSetpoint
            );
            rotationOverride = Optional.of(shotSetpoint.yaw());
            return shotSetpoint;
        });
        var cmd = NonBlockingCmds.parallel(aimAndShoot, runSerializer)
            .finallyDo(() -> {
                shotSetpoint = ShooterSetpoint.NULL;
                rotationOverride = Optional.empty();
            });
        return logged(cmd, "ShootAtHub (Teleop)");
    }
}
