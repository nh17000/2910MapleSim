package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

/**
 * PearadoxTalonFX is a wrapper around the CTRE TalonFX motor controller. It applies configuration, optimizes CAN bus
 * usage, and provides structured access to telemetry.
 */
public class PearadoxTalonFX extends TalonFX {

    private final BaseStatusSignal[] telemetrySignals;

    /**
     * Constructs a new PearadoxTalonFX with the specified device ID and configuration.
     *
     * @param deviceId CAN device ID for the TalonFX
     * @param config TalonFXConfiguration to apply
     */
    public PearadoxTalonFX(int deviceId, TalonFXConfiguration config) {
        super(deviceId);
        applyConfig(config);

        telemetrySignals = new BaseStatusSignal[] {
            getPosition(), getVelocity(), getMotorVoltage(), getSupplyCurrent(), getStatorCurrent(), getDeviceTemp()
        };

        BaseStatusSignal.setUpdateFrequencyForAll(Constants.LOOP_FREQUENCY, telemetrySignals);

        this.optimizeBusUtilization();

        PhoenixUtil.registerSignals(false, telemetrySignals);
    }

    /**
     * Applies the given configuration to this motor controller, retrying up to 5 times in case of transient failures.
     *
     * @param config TalonFXConfiguration to apply
     */
    public void applyConfig(TalonFXConfiguration config) {
        PhoenixUtil.tryUntilOk(5, () -> this.getConfigurator().apply(config, 0.25));
    }

    /**
     * Retrieves key telemetry from this motor, including a connectivity flag.
     *
     * @return MotorData with position, velocity, voltage, currents, temperature, and isConnected
     */
    public MotorData getData() {
        boolean connected = BaseStatusSignal.isAllGood(telemetrySignals);

        return new MotorData(
                telemetrySignals[0].getValueAsDouble(), // position
                telemetrySignals[1].getValueAsDouble(), // velocity
                telemetrySignals[2].getValueAsDouble(), // voltage
                telemetrySignals[3].getValueAsDouble(), // supply current
                telemetrySignals[4].getValueAsDouble(), // stator current
                telemetrySignals[5].getValueAsDouble(), // temperature
                connected);
    }

    /**
     * Record representing a snapshot of TalonFX telemetry.
     *
     * @param position Sensor position (rotations)
     * @param velocity Sensor velocity (rotations/sec)
     * @param appliedVolts Voltage applied to motor
     * @param supplyCurrent Current drawn from supply (A)
     * @param statorCurrent Current through motor windings (A)
     * @param temperature Temperature in °C
     * @param isConnected True if all status signals are valid and connected
     */
    public record MotorData(
            double position,
            double velocity,
            double appliedVolts,
            double supplyCurrent,
            double statorCurrent,
            double temperature,
            boolean isConnected) {

        public MotorData() {
            this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
        }
    }
}
