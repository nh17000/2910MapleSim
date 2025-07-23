package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import frc.robot.Constants;

public class PearadoxCANRange extends CANrange {
    private final BaseStatusSignal[] telemetrySignals;

    /**
     * Constructs a new PearadoxCANRange with the specified device ID and configuration.
     *
     * @param deviceId CAN device ID for the CANRange
     * @param config CANrangeConfiguration to apply
     */
    public PearadoxCANRange(int deviceId, CANrangeConfiguration config) {
        super(deviceId);

        telemetrySignals = new BaseStatusSignal[] {
            getIsDetected(), getDistance(), getSignalStrength(),
            getAmbientSignal(), getDistanceStdDev(), getMeasurementTime()
        };

        BaseStatusSignal.setUpdateFrequencyForAll(Constants.LOOP_FREQUENCY, telemetrySignals);

        this.optimizeBusUtilization();

        PhoenixUtil.registerSignals(false, telemetrySignals);
    }

    /**
     * Applies the given configuration to this CANRange, retrying up to 5 times in case of transient failures.
     *
     * @param config CANrangeConfiguration to apply
     */
    public void applyConfig(CANrangeConfiguration config) {
        PhoenixUtil.tryUntilOk(5, () -> this.getConfigurator().apply(config, 0.25));
    }

    /**
     * Retrieves key telemetry from this sensor, including a connectivity flag.
     *
     * @return SensorData
     */
    public SensorData getData() {
        boolean connected = BaseStatusSignal.isAllGood(telemetrySignals);

        return new SensorData(
                getIsDetected().getValue(),
                telemetrySignals[1].getValueAsDouble(),
                telemetrySignals[2].getValueAsDouble(),
                telemetrySignals[3].getValueAsDouble(),
                telemetrySignals[4].getValueAsDouble(),
                telemetrySignals[5].getValueAsDouble(),
                connected);
    }

    /** Record representing a snapshot of CANRange telemetry. */
    public record SensorData(
            boolean isDetected,
            double distance,
            double signalStrength,
            double ambientSignal,
            double distanceStdDev,
            double timestamp,
            boolean isConnected) {

        public SensorData() {
            this(false, 0.0, 0.0, 0.0, 0.0, 0.0, false);
        }
    }
}
