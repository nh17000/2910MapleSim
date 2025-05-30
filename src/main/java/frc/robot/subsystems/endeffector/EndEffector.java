package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
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

        intake();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);
    }

    public void intake() {
        System.out.println("i");
        if (isCoral && isHorizontal) {
            intakeHorizontalCoral();
        } else if (isCoral) {
            intakeVerticalCoral();
        } else {
            intakeAlgae();
        }
    }

    public void outtake() {
        if (inputs.hasCoral) {
            outtakeCoral();
        } else if (inputs.hasAlgae) {
            outtakeAlgae();
        }
    }

    public void stop() {
        io.setLeftSpeed(0.0);
        io.setRightSpeed(0.0);
        io.setTopSpeed(0.0);
    }

    private void intakeVerticalCoral() {
        io.setLeftSpeed(0.3);
        io.setRightSpeed(-0.3);
        io.setTopSpeed(-0.3);
    }

    private void intakeHorizontalCoral() {
        io.setLeftSpeed(0.3);
        io.setRightSpeed(-0.3);
        io.setTopSpeed(0.3);
    }

    private void intakeAlgae() {
        io.setLeftSpeed(0.0);
        io.setRightSpeed(0.0);
        io.setTopSpeed(0.8);
    }

    private void outtakeCoral() {
        io.setLeftSpeed(-0.5);
        io.setRightSpeed(0.5);
        io.setTopSpeed(-0.5);
    }

    private void outtakeAlgae() {
        io.setLeftSpeed(0.0);
        io.setRightSpeed(0.0);
        io.setTopSpeed(-0.8);
    }

    public boolean hasCoral() {
        return inputs.hasCoral;
    }

    public boolean hasAlgae() {
        return inputs.hasAlgae;
    }
}
