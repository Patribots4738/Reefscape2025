package frc.robot.util.hardware.phoenix;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfigurator;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.units.measure.Distance;
import frc.robot.util.Constants.CANRangeConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.GeneralHardwareConstants;

public class CANRangeCustom extends CANrange {

    private final CANrangeConfigurator configurator = getConfigurator();
    private double timeoutSeconds = GeneralHardwareConstants.TIMEOUT_SECONDS;

    private final StatusSignal<Distance> distance;
    private final StatusSignal<Boolean> isDetected;

    /**
     * Represents a custom CANRange with additional functionality.
     * Extends the base CANrange class.
     * 
     * @param id the unique identifier for the CANRange
     */
    public CANRangeCustom(int id) {
        this(id, "rio");
    }

    /**
     * Represents a custom CANRange with additional functionality.
     * Extends the base CANrange class.
     * 
     * @param id the unique identifier for the CANRange
     * @param canBus the CAN bus address for the CANRange object
     */
    public CANRangeCustom(int id, String canBus) {
        super(id, canBus);

        if (FieldConstants.IS_REAL) {
            restoreFactoryDefaults();
        }

        distance = getDistance();
        isDetected = getIsDetected();
        if (FieldConstants.IS_REAL) {
            applySignalFrequency(
                CANRangeConstants.RANGE_UPDATE_FREQ_HZ,
                distance,
                isDetected);
            applyParameter(
                 () -> optimizeBusUtilization(0, timeoutSeconds),
                "Optimize Bus Utilization"
            );
        }
    }

    /**
     * Returns the distance as a double value
     * 
     * @return the distance away from the object as a double
     */
    public double getDistanceAsDouble() {
        return distance.getValueAsDouble();
    }

    /**
     * Restores the factory defaults of the Pigeon2.
     * 
     * @return The status code indicating the result of the operation.
     */
    public StatusCode restoreFactoryDefaults() {
        return applyParameter(
            () -> configurator.apply(new CANrangeConfiguration(), timeoutSeconds),
            "Factory Defaults"
        );
    }

    /**
     * Applies a parameter to the CANCoder device and returns the status code.
     *
     * @param configApplication the supplier function that applies the parameter configuration
     * @param name the name of the parameter
     * @return the status code indicating the result of applying the parameter
     */
    public StatusCode applyParameter(Supplier<StatusCode> configApplication, Supplier<StatusCode> refreshConfig, BooleanSupplier parameterCheckSupplier, String name) {
        return DeviceUtil.applyParameter(configApplication, refreshConfig, parameterCheckSupplier, name, getDeviceID());
    }

    /**
     * Applies a parameter to the device configuration without checking the parameter.
     * 
     * @param configApplication the supplier that applies the configuration parameter
     * @param configName the name of the configuration parameter
     * @return the status code indicating the success or failure of the configuration application
     */
    public StatusCode applyParameter(Supplier<StatusCode> configApplication, String configName) {
        return DeviceUtil.applyParameter(configApplication, configName, getDeviceID());
    }

    /**
     * Applies the specified signal frequency to the given status signals for the Pigeon2Custom device.
     *
     * @param frequency The signal frequency to be applied.
     * @param signals The status signals to which the frequency should be applied.
     * @return The status code indicating the success or failure of applying the signal frequency.
     */
    public StatusCode applySignalFrequency(double frequency, BaseStatusSignal... signals) {
        return DeviceUtil.applySignalFrequency(frequency, getDeviceID(), signals);
    }

    /**
     * Refreshes the signals of the CANRange and returns the status code.
     * 
     * @return The status code indicating the success or failure of the signal refresh.
     */
    public StatusCode refreshSignals() {
        return BaseStatusSignal.refreshAll(distance);
    }

}