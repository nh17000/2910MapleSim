package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class AutoAlign {
    private Pose2d targetPose = Pose2d.kZero;

    private Debouncer isAlignedDebouncer = new Debouncer(0.2);

    @Getter
    @AutoLogOutput
    private boolean isForwards = true;

    private Supplier<Pose2d> robotSupplier;

    public AutoAlign(Supplier<Pose2d> robotPoseSupplier) {
        this.robotSupplier = robotPoseSupplier;
    }

    public Command reefAlignLeft(Drive drive) {
        return reefAlign(drive, AlignConstants.LEFT_BRANCH_OFFSET);
    }

    public Command reefAlignMid(Drive drive) {
        return reefAlign(drive, AlignConstants.MID_OFFSET);
    }

    public Command reefAlignRight(Drive drive) {
        return reefAlign(drive, AlignConstants.RIGHT_BRANCH_OFFSET);
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

        return targetPose =
                tagPose.transformBy(new Transform2d(offset, isForwards ? Rotation2d.k180deg : Rotation2d.kZero));
    }

    public Command stationAlign(Drive drive) {
        return new DriveToPose(drive, this::findStationTargetPose, robotSupplier);
    }

    private Pose2d findStationTargetPose() {
        return targetPose = robotSupplier
                .get()
                .nearest(FieldConstants.CORAL_STATIONS)
                .transformBy(new Transform2d(AlignConstants.STATION_OFFSET, Rotation2d.k180deg));
    }

    public Command netAlign(Drive drive) {
        return new DriveToPose(drive, this::findNetTargetPose, robotSupplier);
    }

    private Pose2d findNetTargetPose() {
        Pose2d robotPose = robotSupplier.get();
        boolean onRedHalf = robotPose.getX() > FieldConstants.BARGE_X;

        double targetX = FieldConstants.BARGE_X + AlignConstants.NET_ALIGN_TZ * (onRedHalf ? 1 : -1);

        double minY = RobotContainer.isRedAlliance() ? AlignConstants.NET_RED_Y[0] : AlignConstants.NET_BLUE_Y[0];
        double maxY = RobotContainer.isRedAlliance() ? AlignConstants.NET_RED_Y[1] : AlignConstants.NET_BLUE_Y[1];
        double targetY = MathUtil.clamp(robotPose.getY(), minY, maxY);

        Rotation2d targetTheta = onRedHalf ? Rotation2d.k180deg : Rotation2d.kZero;

        return targetPose = new Pose2d(targetX, targetY, targetTheta);
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
