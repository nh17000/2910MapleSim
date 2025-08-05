package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants.EEState;
import frc.robot.Constants.EndEffectorConstants.GamePieceMode;
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
    private GamePieceMode mode = GamePieceMode.VERTICAL_CORAL;

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
            boolean frontDetected = inputs.frontData.isDetected();
            boolean backDetected = inputs.backData.isDetected();

            state = switch (mode) {
                case VERTICAL_CORAL -> {
                    if (frontDetected && backDetected) yield EEState.OFF;
                    if (frontDetected) yield EEState.INDEXING_FWD;
                    if (backDetected) yield EEState.INDEXING_BWD;
                    yield EEState.VERTICAL_CORAL_INTAKE;
                }
                case HORIZONTAL_CORAL -> hasCoral() ? EEState.INDEXING_HORIZONTAL : EEState.HORIZONTAL_CORAL_INTAKE;
                case ALGAE -> hasAlgae() ? EEState.OFF : EEState.ALGAE_INTAKE;};
        });
    }

    public Command index(BooleanSupplier isForwardsSupplier) {
        return run(() -> {
            boolean isForwards = isForwardsSupplier.getAsBoolean();
            boolean frontTripped = inputs.frontData.isDetected();
            boolean backTripped = inputs.backData.isDetected();

            state = switch (mode) {
                case VERTICAL_CORAL -> {
                    if (isForwards && backTripped) yield EEState.INDEXING_BWD;
                    if (!isForwards && frontTripped) yield EEState.INDEXING_FWD;
                    yield EEState.OFF;
                }
                case HORIZONTAL_CORAL -> EEState.INDEXING_HORIZONTAL;
                case ALGAE -> EEState.OFF;};
        });
    }

    public Command outtake(BooleanSupplier isForwardsSupplier) {
        return run(() -> {
            boolean isForwards = isForwardsSupplier.getAsBoolean();

            state = switch (mode) {
                case VERTICAL_CORAL -> isForwards
                        ? EEState.VERTICAL_CORAL_OUTTAKE_FWD
                        : EEState.VERTICAL_CORAL_OUTTAKE_BWD;
                case HORIZONTAL_CORAL -> EEState.HORIZONTAL_CORAL_OUTTAKE;
                case ALGAE -> EEState.ALGAE_OUTTAKE;};
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
