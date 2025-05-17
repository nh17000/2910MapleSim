package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.PearadoxTalonFX;

public abstract class ArmIOTalonFX implements ArmIO {
    protected PearadoxTalonFX pivotOne;
    protected PearadoxTalonFX pivotTwo;
    protected PearadoxTalonFX pivotThree;

    protected PearadoxTalonFX extensionOne;
    protected PearadoxTalonFX extensionTwo;
    protected PearadoxTalonFX extensionThree;

    protected PearadoxTalonFX wrist;

    protected ArmIOTalonFX() {
        pivotOne = new PearadoxTalonFX(
                ArmConstants.PIVOT_ONE_ID, NeutralModeValue.Brake, ArmConstants.PIVOT_CURRENT_LIMIT, false);
        pivotTwo = new PearadoxTalonFX(
                ArmConstants.PIVOT_TWO_ID, NeutralModeValue.Brake, ArmConstants.PIVOT_CURRENT_LIMIT, false);
        pivotThree = new PearadoxTalonFX(
                ArmConstants.PIVOT_THREE_ID, NeutralModeValue.Brake, ArmConstants.PIVOT_CURRENT_LIMIT, false);

        extensionOne = new PearadoxTalonFX(
                ArmConstants.EXTENSION_ONE_ID, NeutralModeValue.Brake, ArmConstants.EXTENSION_CURRENT_LIMIT, false);
        extensionTwo = new PearadoxTalonFX(
                ArmConstants.EXTENSION_TWO_ID, NeutralModeValue.Brake, ArmConstants.EXTENSION_CURRENT_LIMIT, false);
        extensionThree = new PearadoxTalonFX(
                ArmConstants.EXTENSION_THREE_ID, NeutralModeValue.Brake, ArmConstants.EXTENSION_CURRENT_LIMIT, false);

        wrist = new PearadoxTalonFX(
                ArmConstants.WRIST_ID, NeutralModeValue.Brake, ArmConstants.WRIST_CURRENT_LIMIT, false);

        pivotOne.getConfigurator().apply(ArmConstants.PIVOT_SLOT0_CONFIGS);
        pivotTwo.getConfigurator().apply(ArmConstants.PIVOT_SLOT0_CONFIGS);
        pivotThree.getConfigurator().apply(ArmConstants.PIVOT_SLOT0_CONFIGS);

        extensionOne.getConfigurator().apply(ArmConstants.EXTENSION_SLOT0_CONFIGS);
        extensionTwo.getConfigurator().apply(ArmConstants.EXTENSION_SLOT0_CONFIGS);
        extensionThree.getConfigurator().apply(ArmConstants.EXTENSION_SLOT0_CONFIGS);

        wrist.getConfigurator().apply(ArmConstants.WRIST_SLOT0_CONFIGS);

        pivotTwo.setControl(new Follower(ArmConstants.PIVOT_ONE_ID, false));
        pivotThree.setControl(new Follower(ArmConstants.PIVOT_ONE_ID, false));

        extensionTwo.setControl(new Follower(ArmConstants.EXTENSION_ONE_ID, false));
        extensionThree.setControl(new Follower(ArmConstants.EXTENSION_ONE_ID, false));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.pivotPositionRots = pivotOne.getPosition().getValueAsDouble();
        inputs.pivotVelocityRps = pivotOne.getVelocity().getValueAsDouble();
        inputs.pivotAppliedVolts = pivotOne.getMotorVoltage().getValueAsDouble();
        inputs.pivotStatorCurrent = pivotOne.getStatorCurrent().getValueAsDouble();
        inputs.pivotSupplyCurrent = pivotOne.getSupplyCurrent().getValueAsDouble();

        inputs.extensionPositionRots = extensionOne.getPosition().getValueAsDouble();
        inputs.extensionVelocityRps = extensionOne.getVelocity().getValueAsDouble();
        inputs.extensionAppliedVolts = extensionOne.getMotorVoltage().getValueAsDouble();
        inputs.extensionStatorCurrent = extensionOne.getStatorCurrent().getValueAsDouble();
        inputs.extensionSupplyCurrent = extensionOne.getSupplyCurrent().getValueAsDouble();

        inputs.wristPositionRots = wrist.getPosition().getValueAsDouble();
        inputs.wristVelocityRps = wrist.getVelocity().getValueAsDouble();
        inputs.wristAppliedVolts = wrist.getMotorVoltage().getValueAsDouble();
        inputs.wristStatorCurrent = wrist.getStatorCurrent().getValueAsDouble();
        inputs.wristSupplyCurrent = wrist.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setPivotSetpoint(double setpointRads) {
        pivotOne.setControl(new PositionVoltage(setpointRads));
    }

    @Override
    public void setExtensionSetpoint(double setpointMeters) {
        extensionOne.setControl(new PositionVoltage(setpointMeters));
    }

    @Override
    public void setWristSetpoint(double setpointRads) {
        wrist.setControl(new PositionVoltage(setpointRads));
    }

    @Override
    public void setPivotVolts(double volts) {
        pivotOne.setVoltage(volts);
    }

    @Override
    public void setExtensionVolts(double volts) {
        extensionOne.setVoltage(volts);
    }

    @Override
    public void setWristVolts(double volts) {
        wrist.setVoltage(volts);
    }
}
