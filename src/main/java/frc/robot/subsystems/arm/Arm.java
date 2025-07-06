package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    @AutoLogOutput
    @Getter
    private ArmState state = ArmState.STOWED;

    private static final LoggedTunableNumber tunablePivot = new LoggedTunableNumber("Arm/Tunable Pivot", 0.0);
    private static final LoggedTunableNumber tunableExtension = new LoggedTunableNumber("Arm/Tunable Extension", 0.0);
    private static final LoggedTunableNumber tunableWrist = new LoggedTunableNumber("Arm/Tunable Wrist", 0.0);

    private ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public Arm(ArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        ArmPosition setpoint = state.position;
        if (state == ArmState.TUNABLE) {
            setpoint = new ArmPosition(
                    Units.degreesToRadians(tunablePivot.get()),
                    Units.inchesToMeters(tunableExtension.get()),
                    Units.degreesToRadians(tunableWrist.get()));
        }

        io.setPivotSetpoint(setpoint.getPivotRads() / ArmConstants.PIVOT_P_COEFFICIENT);
        io.setExtensionSetpoint(setpoint.getExtensionMeters() / ArmConstants.EXTENSION_P_COEFFICIENT);
        io.setWristSetpoint(setpoint.getWristRads() / ArmConstants.WRIST_P_COEFFICIENT);

        Logger.recordOutput("Arm/Pivot Setpoint Rots", setpoint.getPivotRads() / ArmConstants.PIVOT_P_COEFFICIENT);
        Logger.recordOutput(
                "Arm/Extension Setpoint Rots", setpoint.getExtensionMeters() / ArmConstants.EXTENSION_P_COEFFICIENT);
        Logger.recordOutput("Arm/Wrist Setpoint Rots", setpoint.getWristRads() / ArmConstants.WRIST_P_COEFFICIENT);
        Logger.recordOutput("Arm/Component Setpoints", state.position.getComponentTransforms());
    }

    public Command applyState(ArmState desiredState) {
        return new InstantCommand(() -> this.state = desiredState, this);
    }

    public Command followStateSupplier(Supplier<ArmState> stateSupplier) {
        return new RunCommand(() -> this.state = stateSupplier.get(), this);
    }

    @AutoLogOutput
    public double getPivotAngleRads() {
        return inputs.pivotData.position() * ArmConstants.PIVOT_P_COEFFICIENT;
    }

    @AutoLogOutput
    public double getExtensionLengthMeters() {
        return inputs.extensionData.position() * ArmConstants.EXTENSION_P_COEFFICIENT;
    }

    @AutoLogOutput
    public double getWristAngleRads() {
        return inputs.wristData.position() * ArmConstants.WRIST_P_COEFFICIENT;
    }

    public ArmPosition getArmPosition() {
        return new ArmPosition(getPivotAngleRads(), getExtensionLengthMeters(), getWristAngleRads());
    }
}
