package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.ArmConstants;
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
}
