package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double pivotPositionRots = 0.0;
        public double pivotVelocityRps = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotStatorCurrent = 0.0;
        public double pivotSupplyCurrent = 0.0;

        public double extensionPositionRots = 0.0;
        public double extensionVelocityRps = 0.0;
        public double extensionAppliedVolts = 0.0;
        public double extensionStatorCurrent = 0.0;
        public double extensionSupplyCurrent = 0.0;

        public double wristPositionRots = 0.0;
        public double wristVelocityRps = 0.0;
        public double wristAppliedVolts = 0.0;
        public double wristStatorCurrent = 0.0;
        public double wristSupplyCurrent = 0.0;
    }

    default void updateInputs(ArmIOInputs inputs) {}

    default void setPivotSetpoint(double setpointRads) {}

    default void setExtensionSetpoint(double setpointMeters) {}

    default void setWristSetpoint(double setpointRads) {}

    default void setPivotVolts(double volts) {}

    default void setExtensionVolts(double volts) {}

    default void setWristVolts(double volts) {}
}
