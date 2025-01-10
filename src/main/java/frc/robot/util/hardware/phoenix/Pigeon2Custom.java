package frc.robot.util.hardware.phoenix;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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

        restoreFactoryDefaults();

        yawSignal = getYaw();
        pitchSignal = getPitch();
        rollSignal = getRoll();
        yawVelocitySignal = getAngularVelocityZWorld();
        pitchVelocitySignal = getAngularVelocityYWorld();
        rollVelocitySignal = getAngularVelocityXWorld();

        setTelemetryPreference(TelemetryPreference.DEFAULT);
        applyParameter(
            () -> optimizeBusUtilization(0, 1.0),
            "Optimize Bus Utilization"
        );
    }

    /**
     * Applies a parameter to the device configuration without checking the parameter.
     * 
     * @param configApplication the supplier that applies the configuration parameter
     * @param configName the name of the configuration parameter
     * @return the status code indicating the success or failure of the configuration application
     */
    private StatusCode applyParameter(Supplier<StatusCode> configApplication, String configName) {
        return DeviceUtil.applyParameter(configApplication, configName, getDeviceID());
    }

    /**
     * Applies the specified signal frequency to the given status signals for the Pigeon2Custom device.
     *
     * @param frequency The signal frequency to be applied.
     * @param signals The status signals to which the frequency should be applied.
     * @return The status code indicating the success or failure of applying the signal frequency.
     */
    private StatusCode applySignalFrequency(double frequency, BaseStatusSignal... signals) {
        return DeviceUtil.applySignalFrequency(frequency, getDeviceID(), signals);
    }

    /**
     * Restores the factory defaults of the Pigeon2.
     * 
     * @return The status code indicating the result of the operation.
     */
    private StatusCode restoreFactoryDefaults() {
        return applyParameter(
            () -> configurator.apply(new Pigeon2Configuration(), 1.0),
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
        return yawSignal.getValue().in(Degrees);
    }

    public double getYawRadians() {
        return yawSignal.getValue().in(Radians);
    }

    public double getPitchDegrees() {
        return pitchSignal.getValue().in(Degrees);
    }

    public double getPitchRadians() {
        return pitchSignal.getValue().in(Radians);
    }

    public double getRollDegrees() {
        return rollSignal.getValue().in(Degrees);
    }

    public double getRollRadians() {
        return rollSignal.getValue().in(Radians);
    }

    public double getYawVelocityDegreesPerSec() {
        return yawVelocitySignal.getValue().in(DegreesPerSecond);
    }

    public double getYawVelocityRadiansPerSec() {
        return yawVelocitySignal.getValue().in(RadiansPerSecond);
    }

    public double getPitchVelocityDegreesPerSec() {
        return pitchVelocitySignal.getValue().in(DegreesPerSecond);
    }

    public double getPitchVelocityRadiansPerSec() {
        return pitchVelocitySignal.getValue().in(RadiansPerSecond);
    }

    public double getRollVelocityDegreesPerSec() {
        return rollVelocitySignal.getValue().in(DegreesPerSecond);
    }

    public double getRollVelocityRadiansPerSec() {
        return rollVelocitySignal.getValue().in(RadiansPerSecond);
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
