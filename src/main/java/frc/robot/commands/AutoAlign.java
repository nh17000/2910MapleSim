package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class AutoAlign {
    private static final Translation2d LEFT_BRANCH_OFFSET =
            new Translation2d(AlignConstants.REEF_ALIGN_TZ, AlignConstants.REEF_ALIGN_LEFT_TX);
    private static final Translation2d RIGHT_BRANCH_OFFSET =
            new Translation2d(AlignConstants.REEF_ALIGN_TZ, AlignConstants.REEF_ALIGN_RIGHT_TX);
    private static final Translation2d MID_OFFSET =
            new Translation2d(AlignConstants.REEF_ALIGN_TZ, AlignConstants.REEF_ALIGN_MID_TX);
    private static final Translation2d STATION_OFFSET =
            new Translation2d(AlignConstants.STATION_ALIGN_TZ, AlignConstants.STATION_ALIGN_TX);

    private Pose2d targetPose = Pose2d.kZero;

    private Debouncer isAlignedDebouncer = new Debouncer(0.2);

    @Getter
    @AutoLogOutput
    private boolean isForwards = true;

    @Setter
    private Supplier<Pose2d> robotSupplier;

    public AutoAlign(Supplier<Pose2d> robotPoseSupplier) {
        this.robotSupplier = robotPoseSupplier;
    }

    public Command reefAlignLeft(Drive drive) {
        return reefAlign(drive, LEFT_BRANCH_OFFSET);
    }

    public Command reefAlignMid(Drive drive) {
        return reefAlign(drive, MID_OFFSET);
    }

    public Command reefAlignRight(Drive drive) {
        return reefAlign(drive, RIGHT_BRANCH_OFFSET);
    }

    private Command reefAlign(Drive drive, Translation2d offset) {
        return new DriveToPose(drive, () -> findReefTargetPose(robotSupplier.get(), offset), robotSupplier);
    }

    private Pose2d findReefTargetPose(Pose2d currentPose, Translation2d offset) {
        Pose2d tagPose = currentPose.nearest(FieldConstants.REEF_TAGS);

        Rotation2d backwardsRotation = tagPose.getRotation();
        Rotation2d forwardsRotation = backwardsRotation.plus(Rotation2d.k180deg);

        Rotation2d currRot2d = currentPose.getRotation();

        double fwdError = forwardsRotation.minus(currRot2d).getDegrees();
        double bwdError = backwardsRotation.minus(currRot2d).getDegrees();

        isForwards = Math.abs(fwdError) < Math.abs(bwdError);

        targetPose = tagPose.transformBy(new Transform2d(offset, isForwards ? Rotation2d.k180deg : Rotation2d.kZero));

        return targetPose;
    }

    public Command stationAlign(Drive drive) {
        return new DriveToPose(drive, this::findStationTargetPose, robotSupplier);
    }

    private Pose2d findStationTargetPose() {
        targetPose = robotSupplier
                .get()
                .nearest(FieldConstants.CORAL_STATIONS)
                .transformBy(new Transform2d(STATION_OFFSET, Rotation2d.k180deg));
        return targetPose;
    }

    @AutoLogOutput
    public boolean isAligned() {
        Transform2d error = targetPose.minus(robotSupplier.get());

        return error.getTranslation().getNorm() < AlignConstants.ALIGN_TRANSLATION_TOLERANCE
                && error.getRotation().getRadians() < AlignConstants.ALIGN_ROT_TOLERANCE;
    }

    @AutoLogOutput
    public boolean isAlignedDebounced() {
        return isAlignedDebouncer.calculate(isAligned());
    }
}
