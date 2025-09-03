package frc.robot.util.hardware.phoenix;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfigurator;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.units.measure.Distance;
import frc.robot.util.Constants.CANRangeConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.GeneralHardwareConstants;

public class CANRangeCustom extends CANrange {

    private final ProximityParamsConfigs proxParamsConfigs = new ProximityParamsConfigs();
    private final FovParamsConfigs FOVParamsConfigs = new FovParamsConfigs();
    private final ToFParamsConfigs tofParamsConfigs = new ToFParamsConfigs();

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
     * Returns whether the object is detected as a boolean
     * 
     * @return whether the object is detected as a boolean
     */
    public boolean getIsDetectedAsBoolean() {
        return isDetected.getValue();
    }

    /**
     * Restores the factory defaults of the CANRange.
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
     * Applies a parameter to the CANRange device and returns the status code.
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
     * Applies the specified signal frequency to the given status signals for the CANRange device.
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

    /**
     * Configures the paramaters of the FOV in which things are detected.
     * 
     * @param CenterX The center of the FOV on the x plane.
     * @param CenterY The center of the FOV on the y plane.
     * @param RangeX The range at which the FOV spans on the x plane.
     * @param RangeY The range at which the FOV spans on the y plane.
    */
    public StatusCode configFOVParams(double CenterX, double CenterY, double RangeX, double RangeY) {
        FOVParamsConfigs.FOVCenterX = CenterX;
        FOVParamsConfigs.FOVCenterY = CenterY;
        FOVParamsConfigs.FOVRangeX = RangeX;
        FOVParamsConfigs.FOVRangeY = RangeY;
        return 
        applyParameter(
            () -> configurator.apply(FOVParamsConfigs, timeoutSeconds),
            () -> configurator.refresh(FOVParamsConfigs,timeoutSeconds), 
            () -> FOVParamsConfigs.FOVCenterX == CenterX 
                && FOVParamsConfigs.FOVCenterY == CenterY 
                && FOVParamsConfigs.FOVRangeX == RangeX
                && FOVParamsConfigs.FOVRangeY == RangeY, 
            "Fov Params"
        );
    }

    /**
     * Configures the paramaters of the proximity at which things are detected.
     * 
     * @param minValid The minimum signal strength for a valid measurement.
     * @param hysteresis Used to account for dips above and below the set threshold.
     * @param threshold Threshold at which an object will be detected.
     */
    public StatusCode configProxParams(double minValid, double hysteresis, double threshold) {
        proxParamsConfigs.MinSignalStrengthForValidMeasurement = minValid;
        proxParamsConfigs.ProximityHysteresis = hysteresis;
        proxParamsConfigs.ProximityThreshold = threshold;
        return 
        applyParameter(
            () -> configurator.apply(proxParamsConfigs,timeoutSeconds),
            () -> configurator.refresh(proxParamsConfigs,timeoutSeconds),
            () -> proxParamsConfigs.MinSignalStrengthForValidMeasurement == minValid 
                && proxParamsConfigs.ProximityHysteresis == hysteresis 
                && proxParamsConfigs.ProximityThreshold == threshold,
            "Proximity Params"
        );
    }

    /**
     * Time of Flight paramaters.
     * 
     * @param frequency The update frequency.
     * @param updateModeValue The set update mode either LongRangeUserFreq, ShortRange100Hz, or ShortRangeUserFreq.
    */
    public StatusCode configToFParams(double frequency, UpdateModeValue updateModeValue) {
        tofParamsConfigs.UpdateFrequency = frequency;
        tofParamsConfigs.UpdateMode = updateModeValue;
        return 
        applyParameter(
            () -> configurator.apply(tofParamsConfigs, timeoutSeconds),
            () -> configurator.refresh(tofParamsConfigs, timeoutSeconds),
            () -> tofParamsConfigs.UpdateFrequency == frequency
                && tofParamsConfigs.UpdateMode == updateModeValue,
            "ToF Params"
        );
    }
}