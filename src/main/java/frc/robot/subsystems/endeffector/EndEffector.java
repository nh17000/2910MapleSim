package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants.EEState;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
    @AutoLogOutput
    @Getter
    private EEState state = EEState.OFF;

    @AutoLogOutput
    @Getter
    @Setter
    private boolean isCoral = true;

    @AutoLogOutput
    @Getter
    @Setter
    private boolean isHorizontal = false;

    private EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    public EndEffector(EndEffectorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);

        io.setLeftSpeed(state.leftVolts);
        io.setRightSpeed(state.rightVolts);
        io.setTopSpeed(state.topVolts);
    }

    public void intake() {
        if (hasCoral() || hasAlgae()) {
            state = EEState.OFF;
        } else if (isCoral && isHorizontal) {
            state = EEState.HORIZONTAL_CORAL_INTAKE;
        } else if (isCoral) {
            state = EEState.VERTICAL_CORAL_INTAKE;
        } else {
            state = EEState.ALGAE_INTAKE;
        }
    }

    public void outtake(boolean isForwards) {
        if (isCoral && isForwards) {
            state = EEState.VERTICAL_CORAL_OUTTAKE_FWD;
        } else if (isCoral) {
            state = EEState.VERTICAL_CORAL_OUTTAKE_BWD;
        } else {
            state = EEState.ALGAE_OUTTAKE;
        }
    }

    public boolean hasCoral() {
        return inputs.hasCoral;
    }

    public boolean hasAlgae() {
        return inputs.hasAlgae;
    }
}
