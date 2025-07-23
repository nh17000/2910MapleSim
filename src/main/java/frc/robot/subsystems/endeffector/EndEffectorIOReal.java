package frc.robot.subsystems.endeffector;

import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOReal extends EndEffectorIOTalonFX {
    public EndEffectorIOReal() {}

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        super.updateInputs(inputs);

        inputs.hasCoral = frontCANRange.getData().distance() < EndEffectorConstants.TRANSLATIONAL_TOLERANCE;
        inputs.hasAlgae = frontCANRange.getData().distance() < EndEffectorConstants.TRANSLATIONAL_TOLERANCE;
    }
}
