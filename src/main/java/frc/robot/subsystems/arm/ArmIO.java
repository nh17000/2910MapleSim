package frc.robot.subsystems.arm;

import frc.robot.util.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    static class ArmIOInputs {
        public MotorData pivotData = new MotorData();
        public MotorData extensionData = new MotorData();
        public MotorData wristData = new MotorData();
    }

    default void updateInputs(ArmIOInputs inputs) {}

    default void setPivotSetpoint(double setpointRots) {}

    default void setExtensionSetpoint(double setpointRots) {}

    default void setWristSetpoint(double setpointRots) {}

    default void setPivotVolts(double volts) {}

    default void setExtensionVolts(double volts) {}

    default void setWristVolts(double volts) {}
}
