package frc.robot.subsystems.arm;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.TiltedElevatorSim;
import edu.wpi.first.wpilibj.simulation.VariableLengthArmSim;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmIOSim extends ArmIOTalonFX {
    private final TalonFXSimState pivotSimState;
    private final TalonFXSimState extensionSimState;
    private final TalonFXSimState wristSimState;

    private final VariableLengthArmSim variableLengthArmSim = new VariableLengthArmSim(
            ArmConstants.PIVOT_MOTORS,
            ArmConstants.PIVOT_GEAR_RATIO,
            calcPivotMOI(0),
            ArmConstants.ARM_SHOULDER_TO_WRIST_LENGTH,
            ArmConstants.PIVOT_MIN_ANGLE,
            ArmConstants.PIVOT_MAX_ANGLE,
            ArmConstants.ARM_MASS_KG,
            true);

    private final TiltedElevatorSim tiltedElevatorSim = new TiltedElevatorSim(
            ArmConstants.EXTENSION_MOTORS,
            ArmConstants.EXTENSION_GEAR_RATIO,
            ArmConstants.ARM_MASS_KG,
            ArmConstants.EXTENSION_DRUM_RADIUS,
            ArmConstants.EXTENSION_MIN_LENGTH,
            ArmConstants.EXTENSION_MAX_LENGTH,
            true);

    private final SingleJointedArmSim wristSim = new SingleJointedArmSim(
            ArmConstants.WRIST_MOTOR,
            ArmConstants.WRIST_GEAR_RATIO,
            ArmConstants.WRIST_MASS_KG,
            ArmConstants.WRIST_LENGTH,
            ArmConstants.WRIST_MIN_ANGLE,
            ArmConstants.WRIST_MAX_ANGLE,
            false,
            ArmConstants.WRIST_STARTING_ANGLE);

    public ArmIOSim() {
        pivotSimState = pivotOne.getSimState();
        extensionSimState = extensionOne.getSimState();
        wristSimState = wrist.getSimState();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        super.updateInputs(inputs);

        updateSim();
    }

    private void updateSim() {
        pivotSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);
        extensionSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);
        wristSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);

        // pivot
        variableLengthArmSim.setInputVoltage(pivotSimState.getMotorVoltage());
        variableLengthArmSim.update(Constants.LOOP_PERIOD);

        pivotSimState.setRawRotorPosition(variableLengthArmSim.getAngleRads() / ArmConstants.PIVOT_P_COEFFICIENT);
        pivotSimState.setRotorVelocity(variableLengthArmSim.getVelocityRadPerSec() / ArmConstants.PIVOT_P_COEFFICIENT);

        // extension
        tiltedElevatorSim.setInput(extensionSimState.getMotorVoltage());
        tiltedElevatorSim.update(Constants.LOOP_PERIOD);

        extensionSimState.setRawRotorPosition(
                tiltedElevatorSim.getPositionMeters() / ArmConstants.EXTENSION_P_COEFFICIENT);
        extensionSimState.setRotorVelocity(
                tiltedElevatorSim.getVelocityMetersPerSecond() / ArmConstants.EXTENSION_P_COEFFICIENT);

        // pivot + extension
        variableLengthArmSim.setCGRadius(
                (tiltedElevatorSim.getPositionMeters() + ArmConstants.ARM_SHOULDER_TO_WRIST_LENGTH) / 2.0);
        variableLengthArmSim.setMOI(calcPivotMOI(tiltedElevatorSim.getPositionMeters()));

        tiltedElevatorSim.setAngleFromHorizontal(variableLengthArmSim.getAngleRads());

        // wrist
        wristSim.setInputVoltage(wristSimState.getMotorVoltage());
        wristSim.update(Constants.LOOP_PERIOD);

        wristSimState.setRawRotorPosition(wristSim.getAngleRads() / ArmConstants.WRIST_P_COEFFICIENT);
        wristSimState.setRotorVelocity(wristSim.getVelocityRadPerSec() / ArmConstants.WRIST_P_COEFFICIENT);
    }

    @AutoLogOutput(key = "Arm/Estimated MOI")
    private double calcPivotMOI(double extensionLength) {
        double m = ArmConstants.ARM_MASS_KG;
        double r = ArmConstants.ARM_SHOULDER_TO_WRIST_LENGTH + extensionLength;
        return 1.0 / 3.0 * m * r * r;
    }
}
