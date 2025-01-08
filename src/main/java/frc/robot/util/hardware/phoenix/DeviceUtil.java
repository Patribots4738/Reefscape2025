package frc.robot.util.hardware.phoenix;

import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;

import frc.robot.util.Constants.GeneralHardwareConstants;

public class DeviceUtil {

    public static final double MAX_ATTEMPTS = GeneralHardwareConstants.SAFE_HARDWARE_MODE ? 20 : 4;
    
    /**
     * Applies a parameter to a device using the provided status code supplier.
     * 
     * @param statusCode The supplier that provides the status code.
     * @param name The name of the parameter.
     * @param deviceId The ID of the device.
     * @return The status code after applying the parameter.
     */
    public static StatusCode applyParameter(Supplier<StatusCode> applySupplier, Supplier<StatusCode> refreshSupplier, BooleanSupplier parameterCheckSupplier, String name, int deviceId) {
        StatusCode applyStatus = StatusCode.TaskIsBusy;
        StatusCode refreshStatus = StatusCode.TaskIsBusy;
        boolean parameterCheck = false;
        for (int i = 1; i <= MAX_ATTEMPTS; i++) {
            applyStatus = applySupplier.get();
            refreshStatus = refreshSupplier.get();
            parameterCheck = parameterCheckSupplier.getAsBoolean();
            if (applyStatus.isOK() && refreshStatus.isOK() && parameterCheck) {
                System.err.println("Successfully applied " + name + " to device " + deviceId + " on attempt " + i);
                return applyStatus;
            } else {
                System.err.println("Failed to apply " + name + " to device " + deviceId + " on attempt " + i);
            }
        }
        return applyStatus;
    }

    /**
     * Applies a parameter to a device without checking the parameter and returns the status code
     *
     * @param applySupplier the supplier that applies the parameter
     * @param name the name of the parameter
     * @param deviceId the ID of the device
     * @return the status code after applying the parameter
     */
    public static StatusCode applyParameter(Supplier<StatusCode> applySupplier, String name, int deviceId) {
        return applyParameter(applySupplier, () -> StatusCode.OK, () -> true, name, deviceId);
    }

    /**
     * Applies the specified signal frequency to the given device and signals.
     * 
     * @param frequency The signal frequency to apply.
     * @param deviceId The ID of the device.
     * @param signals The signals to apply the frequency to.
     * @return The status code indicating the result of the operation.
     */
    public static StatusCode applySignalFrequency(double frequency, int deviceId, BaseStatusSignal... signals) {
        return applyParameter(
            () -> BaseStatusSignal.setUpdateFrequencyForAll(frequency, signals),
            frequency + " Hz Signal Frequency",
            deviceId
        );
    }
}
