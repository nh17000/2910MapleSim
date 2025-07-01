package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisualizerConstants;
import frc.robot.util.AlgaeHandler;
import java.util.function.Supplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffectorIOSim extends EndEffectorIOTalonFX {
    private boolean hasCoral = true;
    private boolean hasAlgae = false;

    private boolean horizontal = false;

    private Timer intakingTimer = new Timer();
    private Timer droppingTimer = new Timer();

    private Pose3d intookGamePiecePrevPose = Pose3d.kZero;

    private final FlywheelSim rollerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    EndEffectorConstants.LR_MOTOR,
                    EndEffectorConstants.LR_MOI,
                    1.0 / EndEffectorConstants.LR_GEAR_RATIO),
            EndEffectorConstants.LR_MOTOR);

    @AutoLogOutput(key = "EndEffector/Roller Theta")
    private double rollerAngularPosition = 0.0;

    @AutoLogOutput(key = "EndEffector/Coral Pos")
    private double coralPos = 0.0;

    private static final double MAX_POS = 0.15;
    private static final double INTAKE_POS = 0.1;
    private static final double MIN_POS = -0.05;
    private static final double POS_TOLERANCE = 0.01;

    private final IntakeSimulation coralGroundIntakeSim;
    private final IntakeSimulation algaeGroundIntakeSim;

    private final TalonFXSimState leftSimState;

    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    private Supplier<Transform3d> wristTransformSupplier;

    public EndEffectorIOSim(
            AbstractDriveTrainSimulation driveSimulation, Supplier<Transform3d> wristTransformSupplier) {

        coralGroundIntakeSim = IntakeSimulation.OverTheBumperIntake(
                "Coral", driveSimulation, Inches.of(12), Inches.of(6), IntakeSimulation.IntakeSide.FRONT, 1);
        algaeGroundIntakeSim = IntakeSimulation.OverTheBumperIntake(
                "Algae", driveSimulation, Inches.of(12), Inches.of(6), IntakeSimulation.IntakeSide.FRONT, 1);

        this.poseSupplier = driveSimulation::getSimulatedDriveTrainPose;
        this.chassisSpeedsSupplier = driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative;
        this.wristTransformSupplier = wristTransformSupplier;

        leftSimState = left.getSimState();
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        super.updateInputs(inputs);

        updateRollerSim(Constants.LOOP_PERIOD);
        updateGamePieces(inputs.leftData.appliedVolts(), inputs.topData.appliedVolts());

        inputs.hasCoral = hasCoral; // && MathUtil.isNear(0.0, coralPos, 0.15);
        inputs.hasAlgae = hasAlgae;
    }

    private void updateRollerSim(double dtSeconds) {
        leftSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);
        rollerSim.setInputVoltage(leftSimState.getMotorVoltage());
        rollerSim.update(dtSeconds);

        double deltaTheta = dtSeconds * Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec());
        rollerAngularPosition += deltaTheta;
        if (hasCoral) {
            coralPos = MathUtil.clamp(coralPos + EndEffectorConstants.LR_RADIUS * deltaTheta * 20, MIN_POS - POS_TOLERANCE, MAX_POS + POS_TOLERANCE);
        } else {
            coralPos = 0.0;
        }

        leftSimState.setRawRotorPosition(
                Units.radiansToRotations(rollerAngularPosition) * EndEffectorConstants.LR_GEAR_RATIO);
        leftSimState.setRotorVelocity(
                Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec()) * EndEffectorConstants.LR_GEAR_RATIO);
    }

    private void updateGamePieces(double leftVolts, double topVolts) {
        if (Math.abs(leftVolts) < 0.1) { // algae
            if (topVolts > 0.5) { // algae intake
                if (getHeldAlgaeTransform().getMeasureZ().lt(Inches.of(16))) {
                    // intake algae on the ground
                    algaeGroundIntakeSim.startIntake();
                } else { // intake algae on the reef
                    algaeGroundIntakeSim.stopIntake();
                    intakeReefAlgae();
                }
            } else if (topVolts < -0.5) { // algae outtake
                shootAlgae();
            }
        } else { // coral
            horizontal = topVolts > 0.5; // intake horizontally if top rollers running

            if (leftVolts < -0.5) { // coral intake
                dropFromCoralStation(); // drop coral from the station

                if (getHeldCoralTransform().getMeasureZ().lt(Inches.of(8))) {
                    // intake coral on the ground
                    coralGroundIntakeSim.startIntake();
                } else { // intake coral in the air
                    coralGroundIntakeSim.stopIntake();
                    // intakeCoralProjectiles();
                }
            } else if (leftVolts > -0.5) { // coral outtake
                coralGroundIntakeSim.stopIntake();
            }
        }

        if (hasCoral != coralGroundIntakeSim.getGamePiecesAmount() > 0) {
            if (!hasCoral) setHasCoral(true); // game piece intook
            else coralGroundIntakeSim.addGamePieceToIntake(); // preload
        }

        if (hasAlgae != algaeGroundIntakeSim.getGamePiecesAmount() > 0) {
            if (!hasAlgae) hasAlgae = true; // game piece intook
            else algaeGroundIntakeSim.addGamePieceToIntake(); // preload
        }

        if (coralPos > MAX_POS - POS_TOLERANCE) {
            shootCoral(false);
        } else if (coralPos < MIN_POS + POS_TOLERANCE) {
            shootCoral(true);
        }

        visualizeGamePieces();
    }

    private void intakeCoralProjectiles() {
        if (hasCoral) return; // max 1 coral

        Pose3d intakePose = getHeldCoralPose(); // the end effector is the intake

        var iterator = SimulatedArena.getInstance().gamePieceLaunched().iterator();
        while (iterator.hasNext()) {
            var gamePiece = iterator.next();
            if (gamePiece instanceof ReefscapeCoralOnFly) {
                if (checkTolerance(intakePose.minus(gamePiece.getPose3d()))) {
                    iterator.remove();
                    intookGamePiecePrevPose = gamePiece.getPose3d();
                    setHasCoral(true);
                    intakingTimer.restart();
                    break;
                }
            }
        }
    }

    private void intakeReefAlgae() {
        if (hasAlgae) return; // max 1 algae

        AlgaeHandler.getInstance().intake(getHeldAlgaePose()).ifPresent(algae -> {
            intookGamePiecePrevPose =
                    new Pose3d(algae.getTranslation(), getHeldAlgaePose().getRotation());
            hasAlgae = true;
            intakingTimer.restart();
        });
    }

    private static boolean checkTolerance(Transform3d difference) {
        return difference.getTranslation().getNorm() < EndEffectorConstants.TRANSLATIONAL_TOLERANCE;
    }

    private void visualizeGamePieces() {
        Logger.recordOutput(
                "FieldSimulation/Held Coral", hasCoral ? interpolateHeldPose(getHeldCoralPose()) : Pose3d.kZero);
        Logger.recordOutput(
                "FieldSimulation/Held Algae", hasAlgae ? interpolateHeldPose(getHeldAlgaePose()) : Pose3d.kZero);
    }

    private Pose3d interpolateHeldPose(Pose3d targetPose) {
        if (intakingTimer.isRunning()) {
            if (intakingTimer.get() > EndEffectorConstants.INTAKING_TIME) {
                intakingTimer.stop();
                intakingTimer.reset();
            } else {
                return intookGamePiecePrevPose.interpolate(
                        targetPose, intakingTimer.get() / EndEffectorConstants.INTAKING_TIME);
            }
        }
        return targetPose;
    }

    private Transform3d getHeldCoralTransform() {
        return wristTransformSupplier
                .get()
                .plus(VisualizerConstants.CORAL_TRANSFORM)
                .plus(new Transform3d(coralPos, 0, 0, Rotation3d.kZero));
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

    private void shootCoral(boolean forwards) {
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
                        MetersPerSecond.of(horizontal || !forwards ? 2 : -2),
                        // The coral is ejected at this angle
                        eeTransform.getRotation().getMeasureAngle()));

        setHasCoral(false);
        coralGroundIntakeSim.obtainGamePieceFromIntake();
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
        algaeGroundIntakeSim.obtainGamePieceFromIntake();
    }

    private void dropFromCoralStation() {
        if (hasCoral) return;
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

    private void setHasCoral(boolean flag) {
        if (hasCoral != flag && flag) {
            coralPos = INTAKE_POS;
        }
        hasCoral = flag;
    }
}
