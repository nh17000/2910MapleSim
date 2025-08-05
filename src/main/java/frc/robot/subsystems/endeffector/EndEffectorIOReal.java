package frc.robot.subsystems.endeffector;

public class EndEffectorIOReal extends EndEffectorIOTalonFX {
    public EndEffectorIOReal() {}

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        super.updateInputs(inputs);

        // todo: implement
        inputs.hasCoral = inputs.frontData.isDetected() || inputs.backData.isDetected();
        inputs.hasAlgae = inputs.topData.statorCurrent() > 30;
    }
}
