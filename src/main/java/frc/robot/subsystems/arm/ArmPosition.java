package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.Constants.VisualizerConstants;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;

@AllArgsConstructor
@NoArgsConstructor
public class ArmPosition {
    @Getter
    private double pivotRads;

    @Getter
    private double extensionMeters;

    @Getter
    private double wristRads;

    public static ArmPosition getPreset(int level, boolean facingReef) {
        if (facingReef) {
            if (level == 1) {
                return ArmState.L1.position;
            }
            if (level == 2) {
                return ArmState.L2.position;
            }
            if (level == 3) {
                return ArmState.L3.position;
            }
            if (level == 4) {
                return ArmState.L4.position;
            }
        } else {
            if (level == 2) {
                return ArmState.L2_BACKWARDS.position;
            }
            if (level == 3) {
                return ArmState.L3_BACKWARDS.position;
            }
            if (level == 4) {
                return ArmState.L4_BACKWARDS.position;
            }
        }

        // should never get here!
        return new ArmPosition();
    }

    public ArmPosition constrainedToLimits() {
        ArmPosition output = new ArmPosition();

        output.pivotRads = this.pivotRads;
        while (output.pivotRads > ArmConstants.PIVOT_MAX_ANGLE) {
            output.pivotRads -= 360.0;
        }
        while (output.pivotRads < ArmConstants.PIVOT_MIN_ANGLE) {
            output.pivotRads += 360.0;
        }

        output.extensionMeters = this.extensionMeters;
        if (output.extensionMeters > ArmConstants.EXTENSION_MAX_LENGTH) {
            output.extensionMeters = ArmConstants.EXTENSION_MAX_LENGTH;
        }
        if (output.extensionMeters < ArmConstants.EXTENSION_MIN_LENGTH) {
            output.extensionMeters = ArmConstants.EXTENSION_MIN_LENGTH;
        }

        output.wristRads = this.wristRads;
        while (output.wristRads > ArmConstants.WRIST_MAX_ANGLE) {
            output.wristRads -= 360.0;
        }
        while (output.wristRads < ArmConstants.WRIST_MIN_ANGLE) {
            output.wristRads += 360.0;
        }

        return output;
    }

    public Transform3d[] getComponentTransforms() {
        Transform3d stage0 = new Transform3d(VisualizerConstants.STAGE0_ZERO, new Rotation3d(0, -pivotRads, 0));
        Transform3d stage1 = stage0.plus(new Transform3d(extensionMeters * 0.5, 0, 0, Rotation3d.kZero));
        Transform3d stage2 = stage1.plus(new Transform3d(extensionMeters * 0.5, 0, 0, Rotation3d.kZero));
        Transform3d wrist = stage2.plus(new Transform3d(
                VisualizerConstants.WRIST_OFFSET,
                new Rotation3d(0, -wristRads + ArmConstants.WRIST_STARTING_ANGLE, 0)));

        return new Transform3d[] {stage0, stage1, stage2, wrist};
    }

    public Transform3d getWristTransform() {
        return getComponentTransforms()[3];
    }

    public Pose2d getEndEffectorPose() {
        Transform3d wrist = getWristTransform();
        return new Pose2d(wrist.getX(), wrist.getZ(), Rotation2d.fromRadians(wristRads));
    }

    public static ArmPosition generateArmPosition(Pose2d endEffectorPose) {
        // make position relative to pivot
        endEffectorPose = endEffectorPose.transformBy(new Transform2d(
                VisualizerConstants.STAGE0_ZERO.getX(), VisualizerConstants.STAGE0_ZERO.getZ(), new Rotation2d()));

        // angle of the straight line pointing to the desired point
        double straightAngleRadians = Math.atan2(endEffectorPose.getY(), endEffectorPose.getX());

        // length of extension, calculate because hypotenuse formed by two legs of arm is equivalent to hypotenuse from
        // pivot to point
        double extensionLengthMeters = Math.sqrt(Math.pow(endEffectorPose.getX(), 2)
                + Math.pow(endEffectorPose.getY(), 2)
                - Math.pow(ArmConstants.ARM_SHOULDER_TO_WRIST_LENGTH, 2));

        // angle that just the triangle of the arm forms
        double angleOffsetRadians = Math.atan2(extensionLengthMeters, ArmConstants.ARM_SHOULDER_TO_WRIST_LENGTH);

        // apply offset
        double shoulderAngleDegrees = Math.toDegrees(straightAngleRadians) - Math.toDegrees(angleOffsetRadians) + 90;

        return new ArmPosition(
                Units.degreesToRadians(shoulderAngleDegrees),
                extensionLengthMeters,
                endEffectorPose.getRotation().getRadians());
    }
}
