package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.hardware.CANrange;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOReal extends EndEffectorIOTalonFX {
    private CANrange canRange = new CANrange(EndEffectorConstants.CAN_RANGE_ID);

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        super.updateInputs(inputs);

        inputs.hasCoral = canRange.getDistance().getValueAsDouble() < EndEffectorConstants.TRANSLATIONAL_TOLERANCE;
        inputs.hasAlgae = canRange.getDistance().getValueAsDouble() < EndEffectorConstants.TRANSLATIONAL_TOLERANCE;
    }
}
