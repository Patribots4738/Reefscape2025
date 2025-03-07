package frc.robot.util.hardware.phoenix;

import java.util.function.Supplier;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.PigeonConstants;

public class Pigeon2Custom extends Pigeon2 {

    private final Pigeon2Configurator configurator = getConfigurator();

    private TelemetryPreference telemetryPreference;

    private final StatusSignal<Angle> yawSignal;
    private final StatusSignal<Angle> pitchSignal;
    private final StatusSignal<Angle> rollSignal;
    private final StatusSignal<AngularVelocity> yawVelocitySignal;
    private final StatusSignal<AngularVelocity> pitchVelocitySignal;
    private final StatusSignal<AngularVelocity> rollVelocitySignal;

    private double timeoutSeconds = 1.0;

    public enum TelemetryPreference {
        DEFAULT,
        YAW_ONLY // only if we're feeling silly
    }

    public Pigeon2Custom(int id) {
        this(id, "rio");
    }
    
    /**
     * Represents a custom Pigeon2 object with additional functionality.
     *
     * @param id The ID of the Pigeon2 object.
     * @param canBus The CAN bus address of the Pigeon2 object.
     */
    public Pigeon2Custom(int id, String canBus) {
        super(id, canBus);
        
        if (FieldConstants.IS_REAL) {
            restoreFactoryDefaults();
        }

        yawSignal = getYaw();
        pitchSignal = getPitch();
        rollSignal = getRoll();
        yawVelocitySignal = getAngularVelocityZWorld();
        pitchVelocitySignal = getAngularVelocityYWorld();
        rollVelocitySignal = getAngularVelocityXWorld();

        setTelemetryPreference(TelemetryPreference.DEFAULT);
        if (FieldConstants.IS_REAL) {
            applyParameter(
                () -> optimizeBusUtilization(0, timeoutSeconds),
                "Optimize Bus Utilization"
            );
        }
    }

    /**
     * Applies a parameter to the Pigeon2Custom device.
     * 
     * @param configApplication a supplier that provides the configuration application
     * @param configName the name of the configuration
     * @return the status code indicating the result of applying the parameter
     */
    public StatusCode applyParameter(Supplier<StatusCode> configApplication, Supplier<StatusCode> refreshConfig, BooleanSupplier parameterCheckSupplier, String configName) {
        return DeviceUtil.applyParameter(configApplication, refreshConfig, parameterCheckSupplier, configName, getDeviceID());
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
     * Restores the factory defaults of the Pigeon2.
     * 
     * @return The status code indicating the result of the operation.
     */
    public StatusCode restoreFactoryDefaults() {
        return applyParameter(
            () -> configurator.apply(new Pigeon2Configuration(), timeoutSeconds),
            "Factory Defaults"
        );
    }

    /**
     * Sets the telemetry preference for the Pigeon2Custom object.
     * 
     * @param newPreference the new telemetry preference to be set
     * @return true if the telemetry preference was set successfully, false otherwise
     */
    public boolean setTelemetryPreference(TelemetryPreference newPreference) {
        telemetryPreference = newPreference;

        return 
            switch (telemetryPreference) {
                case YAW_ONLY ->
                    applySignalFrequency(
                        PigeonConstants.PIGEON_FAST_UPDATE_FREQ_HZ, 
                        yawSignal,
                        yawVelocitySignal
                    ).isOK() &&
                    applySignalFrequency(
                        0,
                        pitchSignal,
                        pitchVelocitySignal,
                        rollSignal,
                        rollVelocitySignal
                    ).isOK();
                default ->
                    applySignalFrequency(
                        PigeonConstants.PIGEON_FAST_UPDATE_FREQ_HZ, 
                        yawSignal,
                        yawVelocitySignal,
                        pitchSignal,
                        pitchVelocitySignal,
                        rollSignal,
                        rollVelocitySignal
                    ).isOK();
            };
    }

    public double getYawDegrees() {
        return yawSignal.getValueAsDouble();
    }

    public double getYawRadians() {
        return getYawDegrees() * Math.PI / 180;
    }

    public double getPitchDegrees() {
        return pitchSignal.getValueAsDouble();
    }

    public double getPitchRadians() {
        return getPitchDegrees() * Math.PI / 180;
    }

    public double getRollDegrees() {
        return rollSignal.getValueAsDouble();
    }

    public double getRollRadians() {
        return getRollDegrees() * Math.PI / 180;
    }

    public double getYawVelocityDegreesPerSec() {
        return yawVelocitySignal.getValueAsDouble();
    }

    public double getYawVelocityRadiansPerSec() {
        return getYawVelocityDegreesPerSec() * Math.PI / 180;
    }

    public double getPitchVelocityDegreesPerSec() {
        return pitchVelocitySignal.getValueAsDouble();
    }

    public double getPitchVelocityRadiansPerSec() {
        return getPitchVelocityDegreesPerSec() * Math.PI / 180;
    }

    public double getRollVelocityDegreesPerSec() {
        return rollVelocitySignal.getValueAsDouble();
    }

    public double getRollVelocityRadiansPerSec() {
        return getRollVelocityDegreesPerSec() * Math.PI / 180;
    }

    public Rotation2d getYawRotation2d() {
        return Rotation2d.fromDegrees(getYawDegrees());
    }

    public Rotation2d getPitchRotation2d() {
        return Rotation2d.fromDegrees(getPitchDegrees());
    }

    public Rotation2d getRollRotation2d() {
        return Rotation2d.fromDegrees(getRollDegrees());
    }

    /**
     * Refreshes the signals based on the telemetry preference.
     * 
     * @return The status code indicating the success or failure of the signal refresh.
     */
    public StatusCode refreshSignals() {
        return 
            switch(telemetryPreference) {
                case YAW_ONLY ->
                    BaseStatusSignal.refreshAll(
                        yawSignal,
                        yawVelocitySignal
                    );
                default ->
                    BaseStatusSignal.refreshAll(
                        yawSignal,
                        pitchSignal,
                        rollSignal,
                        yawVelocitySignal,
                        pitchVelocitySignal,
                        rollVelocitySignal
                    );
            };
    }

}
