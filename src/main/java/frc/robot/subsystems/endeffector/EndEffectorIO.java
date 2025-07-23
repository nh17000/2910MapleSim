package frc.robot.subsystems.endeffector;

import frc.robot.util.PearadoxCANRange.SensorData;
import frc.robot.util.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorIOInputs {
        public MotorData leftData = new MotorData();
        public MotorData rightData = new MotorData();
        public MotorData topData = new MotorData();

        public SensorData frontData = new SensorData();
        public SensorData frontLeftData = new SensorData();
        public SensorData frontRightData = new SensorData();
        public SensorData backData = new SensorData();

        public boolean hasCoral = false;
        public boolean hasAlgae = false;
    }

    default void updateInputs(EndEffectorIOInputs inputs) {}

    default void setLeftVolts(double volts) {}

    default void setRightVolts(double volts) {}

    default void setTopVolts(double volts) {}
}
