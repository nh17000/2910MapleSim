package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.FieldConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class CoralStationSim {
    public static Pose2d getNearestStation(Pose2d curPose) {
        return curPose.nearest(FieldConstants.CORAL_STATIONS);
    }

    public static void drop(Pose2d stationPose, boolean addRandomness) {
        GamePieceProjectile gamePiece;

        if (addRandomness) {
            gamePiece = new ReefscapeCoralOnFly(
                    stationPose.getTranslation(),
                    new Translation2d(0, Math.random() * 3 - 1.5),
                    new ChassisSpeeds(),
                    stationPose.getRotation().plus(Rotation2d.fromDegrees(Math.random() * 30 - 15)),
                    Meters.of(1),
                    MetersPerSecond.of(2.5 + Math.random() * 5),
                    Degrees.of(-55 + Math.random() * 30));
        } else {
            gamePiece = new ReefscapeCoralOnFly(
                    stationPose.getTranslation(),
                    Translation2d.kZero,
                    new ChassisSpeeds(),
                    stationPose.getRotation(),
                    Meters.of(1),
                    MetersPerSecond.of(5),
                    Degrees.of(-55));
        }

        SimulatedArena.getInstance().addGamePieceProjectile(gamePiece);
    }
}
