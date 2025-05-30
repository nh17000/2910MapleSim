package frc.robot.subsystems.arm;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.TiltedElevatorSim;
import edu.wpi.first.wpilibj.simulation.VariableLengthArmSim;
import frc.robot.Constants.ArmConstants;

public class ArmIOSim extends ArmIOTalonFX {
    private final TalonFXSimState pivotSimState;
    private final TalonFXSimState extensionSimState;
    private final TalonFXSimState wristSimState;

    private final VariableLengthArmSim variableLengthArmSim = new VariableLengthArmSim(
            DCMotor.getKrakenX60(3),
            ArmConstants.PIVOT_GEAR_RATIO,
            calcPivotMOI(0),
            ArmConstants.ARM_SHOULDER_TO_WRIST_LENGTH,
            ArmConstants.PIVOT_MIN_ANGLE,
            ArmConstants.PIVOT_MAX_ANGLE,
            ArmConstants.ARM_MASS_KG,
            true);

    private final TiltedElevatorSim tiltedElevatorSim = new TiltedElevatorSim(
            DCMotor.getKrakenX60(3),
            ArmConstants.EXTENSION_GEAR_RATIO,
            ArmConstants.ARM_MASS_KG,
            ArmConstants.EXTENSION_DRUM_RADIUS,
            ArmConstants.EXTENSION_MIN_LENGTH,
            ArmConstants.EXTENSION_MAX_LENGTH,
            true);

    private final SingleJointedArmSim wristSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), // actually a kraken x44
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
        pivotSimState.setSupplyVoltage(12);
        extensionSimState.setSupplyVoltage(12);
        wristSimState.setSupplyVoltage(12);

        // pivot
        variableLengthArmSim.setInputVoltage(pivotSimState.getMotorVoltage());
        variableLengthArmSim.update(0.02);

        pivotSimState.setRawRotorPosition(
                Units.radiansToRotations(variableLengthArmSim.getAngleRads() * ArmConstants.PIVOT_GEAR_RATIO));
        pivotSimState.setRotorVelocity(
                Units.radiansToRotations(variableLengthArmSim.getVelocityRadPerSec() * ArmConstants.PIVOT_GEAR_RATIO));

        // extension
        tiltedElevatorSim.setInput(extensionSimState.getMotorVoltage());
        tiltedElevatorSim.update(0.02);

        extensionSimState.setRawRotorPosition(
                Units.radiansToRotations(tiltedElevatorSim.getPositionMeters() / ArmConstants.EXTENSION_DRUM_RADIUS)
                        * ArmConstants.EXTENSION_GEAR_RATIO);

        extensionSimState.setRotorVelocity(Units.radiansToRotations(
                        tiltedElevatorSim.getVelocityMetersPerSecond() / ArmConstants.EXTENSION_DRUM_RADIUS)
                * ArmConstants.EXTENSION_GEAR_RATIO);

        // pivot + extension
        variableLengthArmSim.setCGRadius(
                (tiltedElevatorSim.getPositionMeters() + ArmConstants.ARM_SHOULDER_TO_WRIST_LENGTH) / 2.0);

        tiltedElevatorSim.setAngleFromHorizontal(variableLengthArmSim.getAngleRads());

        // wrist
        wristSim.setInputVoltage(wristSimState.getMotorVoltage());
        wristSim.update(0.02);

        wristSimState.setRawRotorPosition(
                Units.radiansToRotations(wristSim.getAngleRads() * ArmConstants.WRIST_GEAR_RATIO));
        wristSimState.setRotorVelocity(
                Units.radiansToRotations(wristSim.getVelocityRadPerSec() * ArmConstants.WRIST_GEAR_RATIO));
    }

    private double calcPivotMOI(double extensionLength) {
        double m = ArmConstants.ARM_MASS_KG;
        double r = ArmConstants.ARM_SHOULDER_TO_WRIST_LENGTH + extensionLength;
        return 1.0 / 3.0 * m * r * r;
    }
}
