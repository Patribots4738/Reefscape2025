package frc.robot.util.hardware.phoenix;

import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.Constants.CANCoderConstants;

public class CANCoderCustom extends CANcoder {

    private final MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();

    private final CANcoderConfigurator configurator = getConfigurator();

    private final StatusSignal<Angle> position;
    private final StatusSignal<Angle> absolutePosition;
    private final StatusSignal<AngularVelocity> velocity;

    private double positionConversionFactor = 1.0;
    private double velocityConversionFactor = 1.0;


    public CANCoderCustom(int id) {
        this(id, "rio");
    }

    /**
     * Represents a custom CANCoder with additional functionality.
     * Extends the base CANCoder class.
     *
     * @param id The unique identifier for the CANCoder.
     * @param canBus The CAN bus to which the CANCoder is connected.
     */
    public CANCoderCustom(int id, String canBus) {
        super(id, canBus);

        restoreFactoryDefaults();

        position = getPosition();
        absolutePosition = getAbsolutePosition();
        velocity = getVelocity();

        applySignalFrequency(
            CANCoderConstants.ENCODER_UPDATE_FREQ_HZ,
            absolutePosition,
            position,
            velocity
        );
        applyParameter(
            () -> optimizeBusUtilization(0, 1.0),
            "Optimize Bus Utilization"
        );

    }

    /**
     * Refreshes the signals of the CANCoder and returns the status code.
     *
     * @return The status code indicating the success or failure of the signal refresh.
     */
    public StatusCode refreshSignals() {
        return BaseStatusSignal.refreshAll(absolutePosition, position, velocity);
    }


    /**
     * Returns the position as a double value.
     * 
     * @return the position as rotations * PCF
     */
    public double getPositionAsDouble() {
        return position.getValueAsDouble() * positionConversionFactor;
    }

    /**
     * Returns the absolute position as a double value.
     *
     * @return the absolute position as rotation from -0.5 to 0.5 * PCF
     */
    public double getAbsolutePositionAsDouble() {
        return absolutePosition.getValueAsDouble() * positionConversionFactor;
    }

    /**
     * Returns the velocity as a double value.
     * 
     * @return the velocity as rps * VCF
     */
    public double getVelocityAsDouble() {
        return velocity.getValueAsDouble() * velocityConversionFactor;
    }

    /**
     * Resets the encoder to the specified position.
     * 
     * @param position the desired position to reset the encoder to, position / PCF = rotations
     * @return the status code indicating the success or failure of the operation
     */
    public StatusCode resetEncoder(double position) {
        return applyParameter(
            () -> setPosition(position / positionConversionFactor),
            "Encoder Reset"
        );
    }

    public StatusCode resetEncoder() {
        return resetEncoder(0);
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
     * Applies the specified signal frequency to the CANCoder device for the given status signals.
     *
     * @param frequency The signal frequency to be applied.
     * @param signals   The status signals to apply the frequency to.
     * @return The status code indicating the success or failure of applying the signal frequency.
     */
    public StatusCode applySignalFrequency(double frequency, BaseStatusSignal... signals) {
        return DeviceUtil.applySignalFrequency(frequency, getDeviceID(), signals);
    }

    /**
     * Configures the magnet sensor of the CANCoderCustom.
     * 
     * @param inverted true if the sensor direction is inverted, false otherwise
     * @param offset the magnet offset value
     * @return the status code indicating the success or failure of the configuration
     */
    public StatusCode configureMagnetSensor(boolean inverted, double offset) {
        SensorDirectionValue sensorDirectionValue = inverted 
            ? SensorDirectionValue.Clockwise_Positive 
            : SensorDirectionValue.CounterClockwise_Positive;
        magnetSensorConfigs.AbsoluteSensorDiscontinuityPoint = 0.5;
        magnetSensorConfigs.MagnetOffset = offset;
        magnetSensorConfigs.SensorDirection = sensorDirectionValue;
        return applyParameter(
            () -> configurator.apply(magnetSensorConfigs, 1.0), 
            () -> configurator.refresh(magnetSensorConfigs, 1.0),
            () -> magnetSensorConfigs.AbsoluteSensorDiscontinuityPoint == 0.5 &&
                  (magnetSensorConfigs.MagnetOffset != 0 ^ offset == 0) &&
                  magnetSensorConfigs.SensorDirection == sensorDirectionValue,
            "Magnet Sensor Configs"
        );
    }

    /**
     * Restores the factory defaults of the CANCoder.
     * 
     * @return The status code indicating the result of the operation.
     */
    public StatusCode restoreFactoryDefaults() {
        return applyParameter(
            () -> configurator.apply(new CANcoderConfiguration(), 1.0),
            "Factory Defaults"
        );
    }

    /**
     * Sets the position conversion factor for the CANCoder.
     * This factor is used to convert the raw position value to a meaningful unit.
     *
     * @param newFactor the new position conversion factor to be set
     */
    public void setPositionConversionFactor(double newFactor) {
        positionConversionFactor = newFactor;
    }

    /**
     * Sets the velocity conversion factor for the CANCoder.
     * This factor is used to convert the raw velocity value from the CANCoder to a meaningful unit.
     *
     * @param newFactor the new velocity conversion factor to be set
     */
    public void setVelocityConversionFactor(double newFactor) {
        velocityConversionFactor = newFactor;
    }

}
