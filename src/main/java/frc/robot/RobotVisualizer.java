package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.VisualizerConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class RobotVisualizer {
    private final LoggedMechanism2d mech2d = new LoggedMechanism2d(Units.inchesToMeters(45), Units.inchesToMeters(90));
    private final LoggedMechanismRoot2d pivotRoot = mech2d.getRoot(
            "Pivot Root", -VisualizerConstants.STAGE0_ZERO.getX(), VisualizerConstants.STAGE0_ZERO.getZ());
    private final LoggedMechanismLigament2d arm = pivotRoot.append(new LoggedMechanismLigament2d(
            "Arm",
            ArmConstants.ARM_SHOULDER_TO_WRIST_LENGTH,
            ArmConstants.PIVOT_MIN_ANGLE,
            10,
            new Color8Bit(Color.kDarkSeaGreen)));
    private final LoggedMechanismLigament2d wrist = arm.append(new LoggedMechanismLigament2d(
            "Wrist",
            ArmConstants.WRIST_LENGTH,
            ArmConstants.WRIST_STARTING_ANGLE,
            4,
            new Color8Bit(Color.kLightSeaGreen)));
    private final LoggedMechanismLigament2d wrist2 = arm.append(new LoggedMechanismLigament2d(
            "Wrist2",
            ArmConstants.WRIST_LENGTH * 1.76,
            ArmConstants.WRIST_STARTING_ANGLE + 54.14,
            4,
            new Color8Bit(Color.kMediumSeaGreen)));

    private DoubleSupplier pivotAngleSupplier;
    private DoubleSupplier extensionLengthSupplier;
    private DoubleSupplier wristAngleSupplier;
    private Supplier<Pose2d> poseSupplier;

    public RobotVisualizer(
            DoubleSupplier pivotAngleSupplier,
            DoubleSupplier extensionLengthSupplier,
            DoubleSupplier wristAngleSupplier,
            Supplier<Pose2d> poseSupplier) {
        this.pivotAngleSupplier = pivotAngleSupplier;
        this.extensionLengthSupplier = extensionLengthSupplier;
        this.wristAngleSupplier = wristAngleSupplier;
        this.poseSupplier = poseSupplier;
    }

    public void periodic() {
        double armAngle = pivotAngleSupplier.getAsDouble();
        double extensionLength = extensionLengthSupplier.getAsDouble();
        double wristAngle = wristAngleSupplier.getAsDouble();

        // Mechanism2d
        arm.setAngle(Units.radiansToDegrees(armAngle));
        arm.setLength(extensionLength + ArmConstants.ARM_SHOULDER_TO_WRIST_LENGTH);
        wrist.setAngle(Units.radiansToDegrees(wristAngle));
        wrist2.setAngle(Units.radiansToDegrees(wristAngle) + 54.14);

        Logger.recordOutput("FieldSimulation/Mechanism Visualizer", mech2d);

        // 3D Components
        Transform3d stage0 = new Transform3d(VisualizerConstants.STAGE0_ZERO, new Rotation3d(0, -armAngle, 0));
        Transform3d stage1 = stage0.plus(new Transform3d(extensionLength * 0.5, 0, 0, Rotation3d.kZero));
        Transform3d stage2 = stage1.plus(new Transform3d(extensionLength * 0.5, 0, 0, Rotation3d.kZero));
        Transform3d wrist = stage2.plus(new Transform3d(
                VisualizerConstants.WRIST_OFFSET,
                new Rotation3d(0, -wristAngle + ArmConstants.WRIST_STARTING_ANGLE, 0)));

        Logger.recordOutput("FieldSimulation/Components", new Transform3d[] {stage0, stage1, stage2, wrist});

        // End effector (temp)
        Transform3d gamePiecePos =
                new Transform3d(RobotContainer.opController.getLeftY() * 0.2, 0, 0, Rotation3d.kZero);

        Pose3d coral = new Pose3d(poseSupplier.get())
                .transformBy(wrist)
                .transformBy(VisualizerConstants.CORAL_TRANSFORM)
                .transformBy(gamePiecePos);
        Pose3d horizCoral = new Pose3d(poseSupplier.get())
                .transformBy(wrist)
                .transformBy(VisualizerConstants.HORIZONTAL_CORAL_TRANSFORM)
                .transformBy(gamePiecePos);
        Pose3d algae =
                new Pose3d(poseSupplier.get()).transformBy(wrist).transformBy(VisualizerConstants.ALGAE_TRANSFORM);

        Logger.recordOutput("FieldSimulation/Held Coral", coral);
        Logger.recordOutput("FieldSimulation/Held Algae", algae);
    }
}
