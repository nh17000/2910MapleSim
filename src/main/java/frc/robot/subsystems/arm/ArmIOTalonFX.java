package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.PearadoxTalonFX;

public abstract class ArmIOTalonFX implements ArmIO {
    protected final PearadoxTalonFX pivotOne;
    protected final PearadoxTalonFX pivotTwo;
    protected final PearadoxTalonFX pivotThree;

    protected final PearadoxTalonFX extensionOne;
    protected final PearadoxTalonFX extensionTwo;
    protected final PearadoxTalonFX extensionThree;

    protected final PearadoxTalonFX wrist;

    protected final MotionMagicVoltage pivotMMRequest = new MotionMagicVoltage(0);
    protected final MotionMagicVoltage extensionMMRequest = new MotionMagicVoltage(0);
    protected final MotionMagicVoltage wristMMRequest = new MotionMagicVoltage(0);

    protected ArmIOTalonFX() {
        pivotOne = new PearadoxTalonFX(ArmConstants.PIVOT_ONE_ID, ArmConstants.getPivotConfig());
        pivotTwo = new PearadoxTalonFX(ArmConstants.PIVOT_TWO_ID, ArmConstants.getPivotConfig());
        pivotThree = new PearadoxTalonFX(ArmConstants.PIVOT_THREE_ID, ArmConstants.getPivotConfig());

        extensionOne = new PearadoxTalonFX(ArmConstants.EXTENSION_ONE_ID, ArmConstants.getExtensionConfig());
        extensionTwo = new PearadoxTalonFX(ArmConstants.EXTENSION_TWO_ID, ArmConstants.getExtensionConfig());
        extensionThree = new PearadoxTalonFX(ArmConstants.EXTENSION_THREE_ID, ArmConstants.getExtensionConfig());

        wrist = new PearadoxTalonFX(ArmConstants.WRIST_ID, ArmConstants.getWristConfig());

        pivotTwo.setControl(new Follower(ArmConstants.PIVOT_ONE_ID, false));
        pivotThree.setControl(new Follower(ArmConstants.PIVOT_ONE_ID, false));

        extensionTwo.setControl(new Follower(ArmConstants.EXTENSION_ONE_ID, false));
        extensionThree.setControl(new Follower(ArmConstants.EXTENSION_ONE_ID, false));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.pivotData = pivotOne.getData();
        inputs.extensionData = extensionOne.getData();
        inputs.wristData = wrist.getData();
    }

    @Override
    public void setPivotSetpoint(double setpointRots) {
        pivotOne.setControl(pivotMMRequest.withPosition(setpointRots));
    }

    @Override
    public void setExtensionSetpoint(double setpointRots) {
        extensionOne.setControl(extensionMMRequest.withPosition(setpointRots));
    }

    @Override
    public void setWristSetpoint(double setpointRots) {
        wrist.setControl(wristMMRequest.withPosition(setpointRots));
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
