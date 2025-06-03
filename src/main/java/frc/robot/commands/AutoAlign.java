package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class AutoAlign {
    private Supplier<Pose2d> poseSupplier;

    private PIDController reefStrafeSpeedController =
            new PIDController(AlignConstants.STRAFE_kP, AlignConstants.REEF_kI, AlignConstants.REEF_kD);
    private PIDController reefForwardSpeedController =
            new PIDController(AlignConstants.FWD_kP, AlignConstants.REEF_kI, AlignConstants.REEF_kD);
    private PIDController reefRotationSpeedController =
            new PIDController(AlignConstants.ROT_REEF_kP, AlignConstants.ROT_REEF_kI, AlignConstants.ROT_REEF_kD);

    private Debouncer isAlignedDebouncer = new Debouncer(0.2);

    private double alignSpeedStrafe = 0;
    private double alignSpeedRotation = 0;
    private double alignSpeedForward = 0;

    private int currentReefAlignTagID = 18; // -1
    private int currentCSAlignTagID = 12; // -1
    private Map<Integer, Pose3d> tagPoses3d = getTagPoses();

    @Getter
    @AutoLogOutput
    private boolean isForwards = true;

    public AutoAlign(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
        reefRotationSpeedController.enableContinuousInput(-180, 180);
    }

    public Map<Integer, Pose3d> getTagPoses() {
        Map<Integer, Pose3d> tagPoses = new HashMap<Integer, Pose3d>();
        for (int tag : FieldConstants.RED_REEF_TAG_IDS) {
            tagPoses.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        for (int tag : FieldConstants.BLUE_REEF_TAG_IDS) {
            tagPoses.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        for (int tag : FieldConstants.RED_CORAL_STATION_TAG_IDS) {
            tagPoses.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        for (int tag : FieldConstants.BLUE_CORAL_STATION_TAG_IDS) {
            tagPoses.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        return tagPoses;
    }

    private Rotation2d getTagAngle(int tagID) {
        if (tagPoses3d.containsKey(tagID)) {
            Pose3d tagPose = tagPoses3d.get(tagID);
            return new Rotation2d(tagPose.getRotation().getZ());
        } else return new Rotation2d(0);
    }

    private Pose2d getTagPose(int tagID) {
        if (tagPoses3d.containsKey(tagID)) {
            Pose3d tagPose = tagPoses3d.get(tagID);
            return tagPose.toPose2d();
        } else return new Pose2d();
    }

    public Rotation2d getAlignAngleReef() {
        if (!DriverStation.isAutonomous()) {
            setReefAlignTagIDtoClosest();
        }

        Rotation2d backwardsRotation = getTagAngle(currentReefAlignTagID);
        Rotation2d forwardsRotation = backwardsRotation.plus(Rotation2d.k180deg);

        Rotation2d currRot2d = poseSupplier.get().getRotation();

        double fwdError = forwardsRotation.minus(currRot2d).getDegrees();
        double bwdError = backwardsRotation.minus(currRot2d).getDegrees();

        isForwards = Math.abs(fwdError) < Math.abs(bwdError);

        return isForwards ? forwardsRotation : backwardsRotation;
    }

    public Rotation2d getAlignAngleStation() {
        currentCSAlignTagID = getClosestAprilTag(
                RobotContainer.isRedAlliance()
                        ? FieldConstants.RED_CORAL_STATION_TAG_IDS
                        : FieldConstants.BLUE_CORAL_STATION_TAG_IDS,
                poseSupplier.get());

        return new Rotation2d(getTagAngle(currentCSAlignTagID).getRadians());
    }

    private int getClosestAprilTag(int[] tagIDs, Pose2d robotPose) {
        double minDistance = Double.POSITIVE_INFINITY;
        int closestTagID = -1;

        // iterates through all tag IDs
        for (int i : tagIDs) {
            if (tagPoses3d.containsKey(i)) {
                Pose3d tagPose = tagPoses3d.get(i);

                // distance between robot pose and april tag
                double distance = tagPose.getTranslation()
                        .toTranslation2d()
                        .minus(robotPose.getTranslation())
                        .getNorm();

                if (distance < minDistance) {
                    closestTagID = i;
                    minDistance = distance;
                }
            }
        }

        return closestTagID;
    }

    public double getTagDist() {
        Transform2d offset = poseSupplier.get().minus(getTagPose(currentReefAlignTagID));

        return Math.sqrt(Math.pow(offset.getX(), 2) + Math.pow(offset.getY(), 2));
    }

    public double getAlignStrafeSpeedPercent(double setPoint, int tagID) {
        Transform2d offset = poseSupplier.get().minus(getTagPose(tagID));

        alignSpeedStrafe = reefStrafeSpeedController.calculate(offset.getY(), setPoint);
        alignSpeedStrafe += AlignConstants.ALIGN_KS * Math.signum(alignSpeedStrafe);
        if (isForwards) alignSpeedStrafe *= -1;

        Logger.recordOutput("Align/Strafe Speed", alignSpeedStrafe);
        Logger.recordOutput("Align/Strafe Setpoint", setPoint);
        Logger.recordOutput("Align/Strafe Error", setPoint - offset.getY());

        return alignSpeedStrafe;
    }

    public double getAlignRotationSpeedPercent(Rotation2d targetAngle2d) {
        double robotAngle = poseSupplier.get().getRotation().getDegrees();
        double targetAngle = targetAngle2d.getDegrees();
        double rotationError = robotAngle - targetAngle;

        alignSpeedRotation = reefRotationSpeedController.calculate(robotAngle, targetAngle);
        alignSpeedRotation += AlignConstants.ROT_KS * Math.signum(alignSpeedRotation);

        Logger.recordOutput("Align/Rotation Speed", alignSpeedRotation);
        Logger.recordOutput("Align/Robot Angle", robotAngle);
        Logger.recordOutput("Align/Rotation Error", rotationError);

        return alignSpeedRotation;
    }

    public double getAlignForwardSpeedPercent(double setPoint, int tagID) {
        Transform2d offset = poseSupplier.get().minus(getTagPose(tagID));

        alignSpeedForward = reefForwardSpeedController.calculate(offset.getX(), setPoint);
        alignSpeedForward += AlignConstants.ALIGN_KS * Math.signum(alignSpeedForward);
        if (isForwards) alignSpeedForward *= -1;

        isAlignedDebounced();

        Logger.recordOutput("Align/Forward Speed", alignSpeedForward);
        Logger.recordOutput("Align/x", offset.getX());
        Logger.recordOutput("Align/y", offset.getY());
        Logger.recordOutput("Align/Fwd Error", offset.getX() - setPoint);

        return alignSpeedForward;
    }

    public int getReefAlignTag() {
        return currentReefAlignTagID;
    }

    public int getStationAlignTag() {
        return currentCSAlignTagID;
    }

    public void setReefAlignTagIDtoClosest() {
        currentReefAlignTagID = getClosestAprilTag(
                RobotContainer.isRedAlliance() ? FieldConstants.RED_REEF_TAG_IDS : FieldConstants.BLUE_REEF_TAG_IDS,
                poseSupplier.get());
    }

    @AutoLogOutput(key = "Align/Error/isAligned")
    public boolean isAligned() {
        return Math.abs(reefForwardSpeedController.getError()) < AlignConstants.FWD_TOLERANCE
                && Math.abs(reefStrafeSpeedController.getError()) < AlignConstants.STRAFE_TOLERANCE
                && Math.abs(reefRotationSpeedController.getError()) < AlignConstants.ALIGN_ROT_TOLERANCE_DEGREES;
    }

    @AutoLogOutput(key = "Align/Error/isAlignedDebounced")
    public boolean isAlignedDebounced() {
        return isAlignedDebouncer.calculate(isAligned());
    }

    private Command reefAlign(Drive drive, double tx) {
        return DriveCommands.joystickDrive(
                drive,
                () -> getAlignForwardSpeedPercent(AlignConstants.REEF_ALIGN_TZ, getReefAlignTag()),
                () -> getAlignStrafeSpeedPercent(tx, getReefAlignTag()),
                () -> getAlignRotationSpeedPercent(getAlignAngleReef()),
                false);
    }

    public Command reefAlignLeft(Drive drive) {
        return reefAlign(drive, AlignConstants.REEF_ALIGN_LEFT_TX);
    }

    public Command reefAlignMid(Drive drive) {
        return reefAlign(drive, AlignConstants.REEF_ALIGN_MID_TX);
    }

    public Command reefAlignRight(Drive drive) {
        return reefAlign(drive, AlignConstants.REEF_ALIGN_RIGHT_TX);
    }

    public Command stationAlign(Drive drive) {
        return DriveCommands.joystickDrive(
                drive,
                () -> getAlignForwardSpeedPercent(AlignConstants.STATION_ALIGN_TZ, getStationAlignTag()),
                () -> getAlignStrafeSpeedPercent(AlignConstants.STATION_ALIGN_TX, getStationAlignTag()),
                () -> getAlignRotationSpeedPercent(getAlignAngleStation()),
                false);
    }
}
