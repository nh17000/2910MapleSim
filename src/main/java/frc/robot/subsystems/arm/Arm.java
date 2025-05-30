package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmState;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    @AutoLogOutput
    @Getter
    @Setter
    private ArmState state = ArmState.STOWED;

    @AutoLogOutput
    private int i = 1;

    private ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public Arm(ArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        io.setPivotSetpoint(getPivotMotorRots(state.position.getPivotRads()));
        io.setExtensionSetpoint(getExtensionMotorRots(state.position.getExtensionMeters()));
        io.setWristSetpoint(getWristMotorRots(state.position.getWristRads()));

        Logger.recordOutput("Arm/Pivot Setpoint Rots", getPivotMotorRots(state.position.getPivotRads()));
        Logger.recordOutput("Arm/Extension Setpoint Rots", getExtensionMotorRots(state.position.getExtensionMeters()));
        Logger.recordOutput("Arm/Wrist Setpoint Rots", getWristMotorRots(state.position.getWristRads()));
    }

    public void incrementArmState() {
        setState(ArmState.values()[i++]);
        if (i == ArmState.values().length) i = 0;
    }

    @AutoLogOutput
    public double getPivotAngleRads() {
        return Units.rotationsToRadians(inputs.pivotData.position()) / ArmConstants.PIVOT_GEAR_RATIO;
    }

    @AutoLogOutput
    public double getExtensionLengthMeters() {
        return (Units.rotationsToRadians(inputs.extensionData.position()) / ArmConstants.EXTENSION_GEAR_RATIO)
                * ArmConstants.EXTENSION_DRUM_RADIUS;
    }

    @AutoLogOutput
    public double getWristAngleRads() {
        return Units.rotationsToRadians(inputs.wristData.position()) / ArmConstants.WRIST_GEAR_RATIO;
    }

    public ArmPosition getArmPosition() {
        return new ArmPosition(getPivotAngleRads(), getExtensionLengthMeters(), getWristAngleRads());
    }

    public static double getPivotMotorRots(double pivotAngleRads) {
        return Units.radiansToRotations(pivotAngleRads) * ArmConstants.PIVOT_GEAR_RATIO;
    }

    public static double getExtensionMotorRots(double extensionLengthMeters) {
        return Units.radiansToRotations(extensionLengthMeters / ArmConstants.EXTENSION_DRUM_RADIUS)
                * ArmConstants.EXTENSION_GEAR_RATIO;
    }

    public static double getWristMotorRots(double wristAngleRads) {
        return Units.radiansToRotations(wristAngleRads) * ArmConstants.WRIST_GEAR_RATIO;
    }
}
