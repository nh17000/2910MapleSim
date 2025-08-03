package frc.robot.subsystems.endeffector;

import frc.robot.Constants.EndEffectorConstants;
import frc.robot.util.PearadoxCANRange;
import frc.robot.util.PearadoxTalonFX;

public abstract class EndEffectorIOTalonFX implements EndEffectorIO {
    protected final PearadoxTalonFX left;
    protected final PearadoxTalonFX right;
    protected final PearadoxTalonFX top;

    protected final PearadoxCANRange frontCANRange;
    protected final PearadoxCANRange frontLeftCANRange;
    protected final PearadoxCANRange frontRightCANRange;
    protected final PearadoxCANRange backCANRange;

    protected EndEffectorIOTalonFX() {
        left = new PearadoxTalonFX(EndEffectorConstants.LEFT_ID, EndEffectorConstants.getLRConfigs());
        right = new PearadoxTalonFX(EndEffectorConstants.RIGHT_ID, EndEffectorConstants.getLRConfigs());
        top = new PearadoxTalonFX(EndEffectorConstants.TOP_ID, EndEffectorConstants.getTopConfigs());

        frontCANRange = new PearadoxCANRange(
                EndEffectorConstants.FRONT_CAN_RANGE_ID, EndEffectorConstants.getCANRangeConfigs());
        frontLeftCANRange = new PearadoxCANRange(
                EndEffectorConstants.FRONT_LEFT_CAN_RANGE_ID, EndEffectorConstants.getCANRangeConfigs());
        frontRightCANRange = new PearadoxCANRange(
                EndEffectorConstants.FRONT_RIGHT_CAN_RANGE_ID, EndEffectorConstants.getCANRangeConfigs());
        backCANRange =
                new PearadoxCANRange(EndEffectorConstants.BACK_CAN_RANGE_ID, EndEffectorConstants.getCANRangeConfigs());
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.leftData = left.getData();
        inputs.rightData = right.getData();
        inputs.topData = top.getData();

        inputs.frontData = frontCANRange.getData();
        inputs.frontLeftData = frontLeftCANRange.getData();
        inputs.frontRightData = frontRightCANRange.getData();
        inputs.backData = backCANRange.getData();
    }

    @Override
    public void setLeftVolts(double volts) {
        left.setVoltage(volts);
    }

    @Override
    public void setRightVolts(double volts) {
        right.setVoltage(volts);
    }

    @Override
    public void setTopVolts(double volts) {
        top.setVoltage(volts);
    }
}
