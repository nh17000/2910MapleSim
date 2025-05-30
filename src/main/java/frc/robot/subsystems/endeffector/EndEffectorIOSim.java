package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisualizerConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

public class EndEffectorIOSim extends EndEffectorIOTalonFX {
    private boolean hasCoral = true;
    private boolean hasAlgae = false;

    private boolean horizontal = false;

    private Timer intakingTimer = new Timer();
    private Timer droppingTimer = new Timer();

    private Pose3d intookGamePiecePrevPose = Pose3d.kZero;
    private List<Pose3d> stagedAlgae = new ArrayList<>(List.of(FieldConstants.REEF_ALGAE_POSES));

    private final IntakeSimulation intakeSimulation;

    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    private Supplier<Transform3d> wristTransformSupplier;

    public EndEffectorIOSim(
            AbstractDriveTrainSimulation driveSimulation, Supplier<Transform3d> wristTransformSupplier) {

        intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                "Coral", driveSimulation, Inches.of(12), Inches.of(6), IntakeSimulation.IntakeSide.FRONT, 1);

        this.poseSupplier = driveSimulation::getSimulatedDriveTrainPose;
        this.chassisSpeedsSupplier = driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative;
        this.wristTransformSupplier = wristTransformSupplier;
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        super.updateInputs(inputs);

        double leftVolts = inputs.leftData.appliedVolts();
        double topVolts = inputs.topData.appliedVolts();

        if (Math.abs(leftVolts) < 0.1) { // algae
            if (topVolts > 0.5) { // algae intake
                intakeAlgae();
            } else if (topVolts < -0.5) { // algae outtake
                shootAlgae();
            }
        } else { // coral
            horizontal = topVolts > 0.5; // intake horizontally if top rollers running

            if (leftVolts > 0.5) { // coral intake
                dropFromCoralStation(); // drop coral from the station

                if (getHeldCoralTransform().getMeasureZ().lt(Inches.of(8))) {
                    // intake coral on the ground
                    intakeSimulation.startIntake();
                } else { // intake coral in the air
                    intakeSimulation.stopIntake();
                    intakeCoralProjectiles();
                }
            } else if (leftVolts < -0.5) { // coral outtake
                shootCoral();
            }
        }

        if (hasCoral != intakeSimulation.getGamePiecesAmount() > 0) {
            if (!hasCoral) hasCoral = true; // game piece intook
            else intakeSimulation.addGamePieceToIntake(); // preload
        }

        inputs.hasCoral = hasCoral;
        inputs.hasAlgae = hasAlgae;

        visualizeHeldGamePiece();
    }

    private void intakeCoralProjectiles() {
        if (hasCoral) return; // max 1 coral

        Pose3d intakePose = getHeldCoralPose(); // the end effector is the intake
        Set<GamePieceProjectile> gamePieceProjectiles =
                SimulatedArena.getInstance().gamePieceLaunched();

        for (GamePieceProjectile gamePiece : gamePieceProjectiles) {
            if (gamePiece instanceof ReefscapeCoralOnFly) {
                if (checkTolerance(intakePose.minus(gamePiece.getPose3d()))) {
                    gamePieceProjectiles.remove(gamePiece);
                    intookGamePiecePrevPose = gamePiece.getPose3d();
                    hasCoral = true;
                    intakingTimer.restart();
                    break;
                }
            }
        }
    }

    private void intakeAlgae() {
        if (hasAlgae) return; // max 1 algae

        Pose3d intakePose = getHeldAlgaePose(); // the end effector is the intake

        for (Pose3d algae : stagedAlgae) {
            if (checkTolerance(intakePose.minus(algae))) {
                stagedAlgae.remove(algae);
                intookGamePiecePrevPose = algae;
                hasAlgae = true;
                intakingTimer.restart();
                break;
            }
        }
    }

    private static boolean checkTolerance(Transform3d difference) {
        return difference.getTranslation().getNorm() < EndEffectorConstants.TRANSLATIONAL_TOLERANCE;
    }

    private void visualizeHeldGamePiece() {
        Logger.recordOutput("FieldSimulation/Pose", new Pose3d(poseSupplier.get()));
        Logger.recordOutput("FieldSimulation/Staged Algae", stagedAlgae.toArray(Pose3d[]::new));

        if (hasCoral) {
            if (intakingTimer.isRunning()) {
                if (intakingTimer.get() > EndEffectorConstants.INTAKING_TIME) {
                    intakingTimer.stop();
                    intakingTimer.reset();
                }
                Logger.recordOutput(
                        "FieldSimulation/Held Coral",
                        intookGamePiecePrevPose.interpolate(
                                getHeldCoralPose(), intakingTimer.get() / EndEffectorConstants.INTAKING_TIME));
            } else {
                Logger.recordOutput("FieldSimulation/Held Coral", getHeldCoralPose());
            }
        } else {
            Logger.recordOutput("FieldSimulation/Held Coral", Pose3d.kZero);
        }

        if (hasAlgae) {
            if (intakingTimer.isRunning()) {
                if (intakingTimer.get() > EndEffectorConstants.INTAKING_TIME) {
                    intakingTimer.stop();
                    intakingTimer.reset();
                }
                Logger.recordOutput(
                        "FieldSimulation/Held Algae",
                        intookGamePiecePrevPose.interpolate(
                                getHeldAlgaePose(), intakingTimer.get() / EndEffectorConstants.INTAKING_TIME));
            } else {
                Logger.recordOutput("FieldSimulation/Held Algae", getHeldAlgaePose());
            }
        } else {
            Logger.recordOutput("FieldSimulation/Held Algae", Pose3d.kZero);
        }
    }

    private Transform3d getHeldCoralTransform() {
        return wristTransformSupplier.get().plus(VisualizerConstants.CORAL_TRANSFORM);
    }

    private Transform3d getHeldHorizontalCoralTransform() {
        return wristTransformSupplier.get().plus(VisualizerConstants.HORIZONTAL_CORAL_TRANSFORM);
    }

    private Transform3d getHeldAlgaeTransform() {
        return wristTransformSupplier.get().plus(VisualizerConstants.ALGAE_TRANSFORM);
    }

    private Pose3d getHeldCoralPose() {
        Pose3d robotPose = new Pose3d(poseSupplier.get());
        Transform3d coralTransform = horizontal ? getHeldHorizontalCoralTransform() : getHeldCoralTransform();
        return robotPose.transformBy(coralTransform);
    }

    private Pose3d getHeldAlgaePose() {
        Pose3d robotPose = new Pose3d(poseSupplier.get());
        return robotPose.transformBy(getHeldAlgaeTransform());
    }

    private void shootCoral() {
        if (!hasCoral) return;

        Transform3d eeTransform = getHeldCoralTransform();

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        // Obtain robot position from drive simulation
                        poseSupplier.get().getTranslation(),
                        // The scoring mechanism is installed at this position on the robot
                        eeTransform.getTranslation().toTranslation2d(),
                        // Obtain robot speed from drive simulation
                        chassisSpeedsSupplier.get(),
                        // Obtain robot facing from drive simulation
                        poseSupplier.get().getRotation(),
                        // The height at which the coral is ejected
                        eeTransform.getMeasureZ(),
                        // The initial speed of the coral
                        MetersPerSecond.of(horizontal ? 2 : -2),
                        // The coral is ejected at this angle
                        eeTransform.getRotation().getMeasureAngle()));

        hasCoral = false;
        intakeSimulation.obtainGamePieceFromIntake();
    }

    private void shootAlgae() {
        if (!hasAlgae) return;

        Transform3d eeTransform = getHeldAlgaeTransform();

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                        // Obtain robot position from drive simulation
                        poseSupplier.get().getTranslation(),
                        // The scoring mechanism is installed at this position on the robot
                        eeTransform.getTranslation().toTranslation2d(),
                        // Obtain robot speed from drive simulation
                        chassisSpeedsSupplier.get(),
                        // Obtain robot facing from drive simulation
                        poseSupplier.get().getRotation(),
                        // The height at which the coral is ejected
                        eeTransform.getMeasureZ(),
                        // The initial speed of the coral
                        MetersPerSecond.of(2.910),
                        // The coral is ejected at this angle
                        eeTransform.getRotation().getMeasureAngle()));

        hasAlgae = false;
    }

    private void dropFromCoralStation() {
        if (hasCoral) {
            return;
        }

        if (droppingTimer.get() > EndEffectorConstants.DROP_COOLDOWN) {
            droppingTimer.stop();
            droppingTimer.reset();
        } else if (droppingTimer.get() > 0) {
            return;
        }

        if (SimulatedArena.getInstance().gamePiecesOnField().size() > 30) return;

        Pose3d eePose = getHeldCoralPose();
        Pose2d nearestCS = eePose.toPose2d().nearest(FieldConstants.CORAL_STATIONS);

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        // Obtain robot position from drive simulation
                        nearestCS.getTranslation(),
                        // The scoring mechanism is installed at this position on the robot
                        new Translation2d(0, Math.random() * 2 - 1),
                        // Obtain robot speed from drive simulation
                        new ChassisSpeeds(),
                        // Obtain robot facing from drive simulation
                        nearestCS.getRotation().plus(Rotation2d.fromDegrees(180 + Math.random() * 60 - 30)),
                        // The height at which the coral is ejected
                        Meters.of(1),
                        // The initial speed of the coral
                        MetersPerSecond.of(Math.random() * 7.5 + 2.5),
                        // The coral is ejected at this angle
                        Degrees.of(-55 + Math.random() * 30)));

        droppingTimer.restart();
    }
}
