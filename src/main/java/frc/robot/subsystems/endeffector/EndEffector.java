package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants.EEState;
import java.util.function.BooleanSupplier;
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

        io.setLeftVolts(state.leftVolts);
        io.setRightVolts(state.rightVolts);
        io.setTopVolts(state.topVolts);
    }

    public Command intake() {
        return run(() -> {
            if (hasCoral() || hasAlgae()) {
                state = isHorizontal ? EEState.INDEXING_HORIZONTAL : EEState.OFF;
            } else if (isCoral) {
                state = isHorizontal ? EEState.HORIZONTAL_CORAL_INTAKE : EEState.VERTICAL_CORAL_INTAKE;
            } else {
                state = EEState.ALGAE_INTAKE;
            }
        });
    }

    public Command index(BooleanSupplier isForwardsSupplier) {
        return run(() -> {
            if (isHorizontal) {
                state = EEState.INDEXING_HORIZONTAL;
            } else {
                boolean isForwards = isForwardsSupplier.getAsBoolean();

                if (isForwards && inputs.backData.isDetected()) {
                    state = EEState.INDEXING_BWD;
                } else if (!isForwards && inputs.frontData.isDetected()) {
                    state = EEState.INDEXING_FWD;
                } else {
                    state = EEState.OFF;
                }
            }
        });
    }

    public Command outtake(BooleanSupplier isForwardsSupplier) {
        return run(() -> {
            if (isCoral) {
                if (isHorizontal) {
                    state = EEState.HORIZONTAL_CORAL_OUTTAKE;
                } else if (isForwardsSupplier.getAsBoolean()) {
                    state = EEState.VERTICAL_CORAL_OUTTAKE_FWD;
                } else {
                    state = EEState.VERTICAL_CORAL_OUTTAKE_BWD;
                }
            } else {
                state = EEState.ALGAE_OUTTAKE;
            }
        });
    }

    public boolean hasCoral() {
        return inputs.hasCoral;
    }

    public boolean hasAlgae() {
        return inputs.hasAlgae;
    }

    public boolean hasGamePiece() {
        return hasCoral() || hasAlgae();
    }
}
