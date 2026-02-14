package robot.subsystems;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.commands.NonBlockingCmds;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import robot.constants.FieldConstants;
import robot.misc.SharedData;
import robot.subsystems.climber.Climber;
import robot.subsystems.drive.SwerveSubsystem;
import robot.subsystems.intake.GroundIntake;
import robot.subsystems.serializer.Serializer;
import robot.subsystems.shooter.Shooter;
import robot.subsystems.shooter.ShooterConsts.ShooterSetpoint;
import robot.subsystems.shooter.math.GroundShotMap;
import robot.subsystems.shooter.math.HubShotMap;
import robot.subsystems.shooter.math.ShooterCalcsKt;
import robot.subsystems.shooter.math.ShotMap;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import static choreo.util.ChoreoAllianceFlipUtil.flip;
import static lib.commands.CmdLogger.logged;
import static robot.subsystems.shooter.ShooterConsts.NULL_SHOOTER_SETPOINT;

@SuppressWarnings("unused")
@RequiredArgsConstructor
public class Superstructure {
    private final List<Rectangle2d>
        hubNoShootZones = List.of(
            new Rectangle2d(new Translation2d(3.8, 8), new Translation2d(5.3, 0))
        ),
        ferryNoShootZones = List.of();
    private final ShotMap
        ferryShotMap = new GroundShotMap(),
        hubShotMap = new HubShotMap();
    private final Translation2d
        blueHub = FieldConstants.Hub.topCenterPoint.toTranslation2d(),
        redHub = flip(blueHub),
        blueTopFerry = new Translation2d(1, 1),
        blueBottomFerry = new Translation2d(1, 7),
        redTopFerry = flip(blueTopFerry),
        redBottomFerry = flip(blueBottomFerry);

    @AutoLogOutput private ShooterSetpoint shotSetpoint = NULL_SHOOTER_SETPOINT;

    private final SwerveSubsystem drive;
    private final GroundIntake groundIntake;
    private final Climber climber;
    private final Shooter shooter;
    private final Serializer serializer;

    private Command shootCmd(
        ShotMap shotMap,
        List<Rectangle2d> noShootZones,
        Supplier<Translation2d> target
    ) {
        var waitForShot = Commands.waitUntil(() -> {
            if (!shotSetpoint.valid()) return false;
            for (var zone: noShootZones) {
                if (zone.contains(drive.getPose().getTranslation())) return false;
            }
            return true;
        });
        var runSerializer = NonBlockingCmds.sequence(
            waitForShot, Commands.waitSeconds(0.5), serializer.runCmd()
        );
        var runShooter = Commands.run(() -> {
            shotSetpoint = ShooterCalcsKt.computeSetpoint(
                target.get(), shotMap, drive.getPose(),
                drive.getFieldSpeeds(), shooter.getVelocity()
            );
            shooter.setTarget(shotSetpoint);
            SharedData.rotOverride = Optional.of(shotSetpoint.yaw());
        }, shooter);
        return NonBlockingCmds.parallel(runShooter, runSerializer)
            .finallyDo(() -> shotSetpoint = NULL_SHOOTER_SETPOINT);
    }

    public Command shootAtHubCmd() {
        var cmd = shootCmd(
            hubShotMap, hubNoShootZones,
            () -> SharedData.redAlliance() ? redHub : blueHub
        );
        return logged(cmd.withName("ShootAtHub"));
    }

    public Command ferryCmd() {
        var cmd = shootCmd(
            ferryShotMap, ferryNoShootZones,
            () -> {
                var topLoc = SharedData.redAlliance() ? redTopFerry : blueTopFerry;
                var bottomLoc = SharedData.redAlliance() ? redBottomFerry : blueBottomFerry;
                return drive.getPose().getY() > 4 ? topLoc : bottomLoc;
            }
        );
        return logged(cmd.withName("FerryShoot"));
    }
}
