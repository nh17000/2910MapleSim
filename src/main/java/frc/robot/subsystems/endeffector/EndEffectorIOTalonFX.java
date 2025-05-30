package frc.robot.subsystems.endeffector;

import frc.robot.Constants.EndEffectorConstants;
import frc.robot.util.PearadoxTalonFX;

public class EndEffectorIOTalonFX implements EndEffectorIO {
    protected final PearadoxTalonFX left;
    protected final PearadoxTalonFX right;
    protected final PearadoxTalonFX top;

    protected EndEffectorIOTalonFX() {
        left = new PearadoxTalonFX(EndEffectorConstants.LEFT_ID, EndEffectorConstants.getLRConfigs());
        right = new PearadoxTalonFX(EndEffectorConstants.RIGHT_ID, EndEffectorConstants.getLRConfigs());
        top = new PearadoxTalonFX(EndEffectorConstants.TOP_ID, EndEffectorConstants.getTopConfigs());
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.leftData = left.getData();
        inputs.rightData = right.getData();
        inputs.topData = top.getData();
    }

    @Override
    public void setLeftSpeed(double speed) {
        left.set(speed);
    }

    @Override
    public void setRightSpeed(double speed) {
        right.set(speed);
    }

    @Override
    public void setTopSpeed(double speed) {
        top.set(speed);
    }
}
