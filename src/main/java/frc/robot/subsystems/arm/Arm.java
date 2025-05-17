package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmState;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private ArmState armState = ArmState.STOWED;

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

        io.setPivotSetpoint(getPivotMotorRots(armState.pivotRads));
        io.setExtensionSetpoint(getExtensionMotorRots(armState.extensionMeters));
        io.setWristSetpoint(getWristMotorRots(armState.wristRads));

        Logger.recordOutput("Arm/State", armState.toString());

        Logger.recordOutput("Arm/Pivot Ang Rads", getPivotAngleRads());
        Logger.recordOutput("Arm/Pivot Setpoint Rots", getPivotMotorRots(armState.pivotRads));

        Logger.recordOutput("Arm/Extension Meters", getExtensionLengthMeters());
        Logger.recordOutput("Arm/Extension Setpoint Rots", getExtensionMotorRots(armState.extensionMeters));

        Logger.recordOutput("Arm/Wrist Ang Rads", getWristAngleRads());
        Logger.recordOutput("Arm/Wrist Setpoint Rots", getWristMotorRots(armState.wristRads));
    }

    public ArmState getState() {
        return armState;
    }

    public void setState(ArmState state) {
        this.armState = state;
    }

    public void incrementArmState() {
        setState(ArmState.values()[i++]);
        if (i == ArmState.values().length) i = 0;
    }

    public double getPivotAngleRads() {
        return Units.rotationsToRadians(inputs.pivotPositionRots) / ArmConstants.PIVOT_GEAR_RATIO;
    }

    public double getExtensionLengthMeters() {
        return (Units.rotationsToRadians(inputs.extensionPositionRots) / ArmConstants.EXTENSION_GEAR_RATIO)
                * ArmConstants.EXTENSION_DRUM_RADIUS;
    }

    public double getWristAngleRads() {
        return Units.rotationsToRadians(inputs.wristPositionRots) / ArmConstants.WRIST_GEAR_RATIO;
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
