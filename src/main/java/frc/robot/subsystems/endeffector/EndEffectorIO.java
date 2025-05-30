package frc.robot.subsystems.endeffector;

import frc.robot.util.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorIOInputs {
        public MotorData leftData = new MotorData();
        public MotorData rightData = new MotorData();
        public MotorData topData = new MotorData();

        public boolean hasCoral = false;
        public boolean hasAlgae = false;
    }

    default void updateInputs(EndEffectorIOInputs inputs) {}

    default void setLeftSpeed(double speed) {}

    default void setRightSpeed(double speed) {}

    default void setTopSpeed(double speed) {}
}
