package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.VisualizerConstants;
import frc.robot.subsystems.arm.ArmPosition;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class RobotVisualizer {
    @AutoLogOutput(key = "FieldSimulation/Mechanism Visualizer")
    private final LoggedMechanism2d mech2d = new LoggedMechanism2d(Units.inchesToMeters(45), Units.inchesToMeters(90));

    private final LoggedMechanismRoot2d pivotRoot = mech2d.getRoot(
            "Pivot Root", -VisualizerConstants.STAGE0_ZERO.getX(), VisualizerConstants.STAGE0_ZERO.getZ());
    private final LoggedMechanismLigament2d arm = pivotRoot.append(new LoggedMechanismLigament2d(
            "Arm",
            ArmConstants.ARM_SHOULDER_TO_WRIST_LENGTH,
            ArmConstants.PIVOT_MIN_ANGLE,
            10,
            new Color8Bit(Color.kDarkSeaGreen)));
    private final LoggedMechanismLigament2d wrist1 = arm.append(new LoggedMechanismLigament2d(
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

    private Supplier<ArmPosition> armPosSupplier;

    public RobotVisualizer(Supplier<ArmPosition> armPositionSupplier) {
        this.armPosSupplier = armPositionSupplier;
    }

    public void periodic() {
        ArmPosition armPos = armPosSupplier.get();
        double armAngle = armPos.getPivotRads();
        double extensionLength = armPos.getExtensionMeters();
        double wristAngle = armPos.getWristRads();

        // Mechanism2d
        arm.setAngle(Units.radiansToDegrees(armAngle));
        arm.setLength(extensionLength + ArmConstants.ARM_SHOULDER_TO_WRIST_LENGTH);
        wrist1.setAngle(Units.radiansToDegrees(wristAngle));
        wrist2.setAngle(Units.radiansToDegrees(wristAngle) + 54.14);

        // 3D Components
        Logger.recordOutput("FieldSimulation/Components", armPos.getComponentTransforms());
    }
}
