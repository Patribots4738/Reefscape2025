package frc.robot.util.hardware.phoenix;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.KrakenMotorConstants;
import frc.robot.util.custom.GainConstants;

public class Kraken extends TalonFX {

    private double targetPosition = 0.0;
    private double targetVelocity = 0.0;
    private double targetPercent = 0.0;

    private double positionConversionFactor = 1.0;
    private double velocityConversionFactor = 1.0;

    private final TalonFXConfigurator configurator = getConfigurator();
    private final TalonFXSimState sim = getSimState();

    private final SlotConfigs slotConfigs = new SlotConfigs();
    private final CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();
    private final MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    private final ClosedLoopGeneralConfigs closedLoopConfigs = new ClosedLoopGeneralConfigs();
    private final FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    private final TorqueCurrentConfigs torqueCurrentConfigs = new TorqueCurrentConfigs();
    private final ClosedLoopRampsConfigs closedLoopRampConfigs = new ClosedLoopRampsConfigs();

    private DCMotorSim motorSimModel;

    private final PositionVoltage positionRequest;
    private final VelocityVoltage velocityRequest;
    private final PositionTorqueCurrentFOC positionTorqueRequest;
    private final VelocityTorqueCurrentFOC velocityTorqueRequest;
    private final VoltageOut voltageRequest;
    private final DutyCycleOut percentRequest;
    private final TorqueCurrentFOC torqueRequest;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Double> percentSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> statorCurrentSignal;   
    private final StatusSignal<Current> torqueCurrentSignal;
    private final StatusSignal<Temperature> temperatureSignal;

    private final GainConstants[] gains;

    private final boolean useFOC;
    private final boolean useTorqueControl;

    private TelemetryPreference telemetryPreference;

    /**
     * Creates new Kraken motor.
     * 
     * @param id ID of Kraken motor
     * @param canBus CANivore Kraken is connected to
     */
    public Kraken(int id, String canBus) {
        this(id, canBus, false, false);
    }

    /**
     * Creates new Kraken motor.
     * 
     * @param id ID of Kraken motor
     */
    public Kraken(int id) {
        this(id, "rio");
    }

    /**
     * Creates a new Kraken motor that can be inverted and use FOC.
     * 
     * @param id ID of Kraken motor
     * @param useFOC uses FOC to enhance motor communication when set to true
     * @param useTorqueControl uses the TalonFX native torque control mode when set to true
     */
    public Kraken(int id, boolean useFOC, boolean useTorqueControl) {
        this(id, "rio", useFOC, useTorqueControl);
    }

    /**
     * Represents a Kraken object that controls a specific hardware component.
     * 
     * @param id The ID of the Kraken object.
     * @param canBus The CAN bus address of the Kraken object.
     * @param useFOC Whether to use Field Oriented Control (FOC) for the Kraken object.
     * @param useTorqueControl Uses the TalonFX native torque control mode when set to true.
     */
    public Kraken(int id, String canBus, boolean useFOC, boolean useTorqueControl) {

        super(id, canBus);

        restoreFactoryDefaults();

        this.useFOC = useFOC;
        this.useTorqueControl = useTorqueControl;

        positionRequest = new PositionVoltage(0).withEnableFOC(useFOC);
        velocityRequest = new VelocityVoltage(0).withEnableFOC(useFOC);
        positionTorqueRequest = new PositionTorqueCurrentFOC(0);
        velocityTorqueRequest = new VelocityTorqueCurrentFOC(0);
        voltageRequest = new VoltageOut(0).withEnableFOC(useFOC);
        percentRequest = new DutyCycleOut(0).withEnableFOC(useFOC);
        torqueRequest = new TorqueCurrentFOC(0);

        gains = new GainConstants[] { new GainConstants(), new GainConstants(), new GainConstants() };

        positionSignal = super.getPosition();
        velocitySignal = super.getVelocity();
        voltageSignal = super.getMotorVoltage();
        percentSignal = super.getDutyCycle();
        supplyCurrentSignal = super.getSupplyCurrent();
        statorCurrentSignal = super.getStatorCurrent();
        torqueCurrentSignal = super.getTorqueCurrent();
        temperatureSignal = super.getDeviceTemp();

        setTelemetryPreference(TelemetryPreference.DEFAULT);
        applyParameter(
            () -> optimizeBusUtilization(0, 1.0),
            "Optimize Bus Utilization"
        );

        register();

    }

    public enum TelemetryPreference {
        DEFAULT,
        NO_ENCODER, // Intake roller, for example
        SWERVE // Swerve module motors
    }

    /**
     * Sets the telemetry preference for the Kraken motor.
     * 
     * @param newPreference the new telemetry preference to set
     * @return true if the telemetry preference was set successfully, false otherwise
     */
    public boolean setTelemetryPreference(TelemetryPreference newPreference) {
        telemetryPreference = newPreference;

        return 
            switch(telemetryPreference) {
                case NO_ENCODER ->
                    applySignalFrequency(
                        KrakenMotorConstants.TALONFX_FAST_UPDATE_FREQ_HZ,
                        voltageSignal,
                        percentSignal
                    ).isOK() &&
                    applySignalFrequency(
                        KrakenMotorConstants.TALONFX_SLOW_UPDATE_FREQ_HZ, 
                        supplyCurrentSignal,
                        statorCurrentSignal,
                        torqueCurrentSignal,
                        temperatureSignal
                    ).isOK() &&
                    applySignalFrequency(
                        0,
                        positionSignal,
                        velocitySignal
                    ).isOK();
                case SWERVE ->
                    applySignalFrequency(
                        KrakenMotorConstants.TALONFX_FAST_UPDATE_FREQ_HZ,
                        positionSignal,
                        velocitySignal,
                        voltageSignal,
                        supplyCurrentSignal,
                        torqueCurrentSignal
                    ).isOK() &&
                    applySignalFrequency(
                        KrakenMotorConstants.TALONFX_SLOW_UPDATE_FREQ_HZ, 
                        temperatureSignal
                    ).isOK() &&
                    applySignalFrequency(
                        0,
                        percentSignal,
                        statorCurrentSignal
                    ).isOK();
                default ->
                    applySignalFrequency(
                        KrakenMotorConstants.TALONFX_FAST_UPDATE_FREQ_HZ, 
                        voltageSignal,
                        percentSignal
                    ).isOK() &&
                    applySignalFrequency(
                        KrakenMotorConstants.TALONFX_MID_UPDATE_FREQ_HZ,
                        positionSignal,
                        velocitySignal
                    ).isOK() &&
                    applySignalFrequency(
                        KrakenMotorConstants.TALONFX_SLOW_UPDATE_FREQ_HZ, 
                        supplyCurrentSignal,
                        statorCurrentSignal,
                        torqueCurrentSignal,
                        temperatureSignal
                    ).isOK();
            };
    }

    public StatusCode setTargetPosition(double position) {
        return setTargetPosition(position, 0, 0);
    }

    public StatusCode setTargetPosition(double position, double feedForward) {
        return setTargetPosition(position, feedForward, 0);
    }

    public StatusCode setTargetPosition(double position, int slot) {
        return setTargetPosition(position, 0, slot);
    }

    /**
     * Sets the target position of the Kraken mechanism.
     * 
     * @param position The desired position of the Kraken mechanism, position / PCF give rotations.
     * @param feedForward The feed forward value for the Kraken mechanism.
     * @param slot The slot number for the Kraken mechanism.
     * @return The status code indicating the success or failure of setting the target position.
     */
    public StatusCode setTargetPosition(double position, double feedForward, int slot) {
        StatusCode status = 
            setControl(
                useTorqueControl 
                    ? positionTorqueRequest
                        .withPosition(position / positionConversionFactor)
                        .withFeedForward(feedForward)
                        .withSlot(slot)
                    : positionRequest
                        .withPosition(position / positionConversionFactor)
                        .withFeedForward(feedForward)
                        .withSlot(slot));
        if (!status.isError()) {
            targetPosition = position;
        } else {
            System.err.println("Failure to set position setpoint");
            System.err.println("Error Code " + status.value + " On Device " + getDeviceID() + " - " + status.getDescription());
        }
        return status;
    }

    public StatusCode setTargetVelocity(double velocity) {
        return setTargetVelocity(velocity, 0, 0);
    }

    public StatusCode setTargetVelocity(double velocity, double feedForward) {
        return setTargetVelocity(velocity, feedForward, 0);
    }

    public StatusCode setTargetVelocity(double velocity, int slot) {
        return setTargetVelocity(velocity, 0, slot);
    }

    /**
     * Sets the target velocity for the Kraken motor controller.
     * 
     * @param velocity The desired velocity in units per second, velocity / VCF gives rps.
     * @param feedForward The feed forward value to be applied.
     * @param slot The slot index for PIDF configuration.
     * @return The status code indicating the success or failure of the operation.
     */
    public StatusCode setTargetVelocity(double velocity, double feedForward, int slot) {
        StatusCode status = 
            setControl(
                useTorqueControl 
                    ? velocityTorqueRequest
                        .withVelocity(velocity / velocityConversionFactor)
                        .withFeedForward(feedForward)
                        .withSlot(slot)
                    : velocityRequest
                        .withVelocity(velocity / velocityConversionFactor)
                        .withFeedForward(feedForward)
                        .withSlot(slot));
        if (!status.isError()) {
            targetVelocity = velocity;
        } else {
            System.err.println("Failure to set velocity setpoint");
            System.err.println("Error Code " + status.value + " On Device " + getDeviceID() + " - " + status.getDescription());
        }
        return status;
    }

    /**
     * Sets the percent output of the Kraken motor controller.
     * 
     * @param percent The desired percent output, ranging from -1.0 to 1.0.
     * @return The status code indicating the success or failure of the operation.
     */
    public StatusCode setPercentOutput(double percent) {
        StatusCode status = setControl(percentRequest.withOutput(percent));
        if (!status.isError()) {
            targetPercent = percent;
        } else {
            System.err.println("Failure to set percent output");
            System.err.println("Error Code " + status.value + " On Device " + getDeviceID() + " - " + status.getDescription());
        }
        return status;
    }

    /**
     * Sets the voltage output of the Kraken device.
     * 
     * @param volts the desired voltage output in volts
     * @return the status code indicating the success or failure of the operation
     */
    public StatusCode setVoltageOutput(double volts) {
        StatusCode status = setControl(voltageRequest.withOutput(volts));
        if (status.isError()) {
            System.err.println("Failure to set voltage output");
            System.err.println("Error Code " + status.value + " On Device " + getDeviceID() + " - " + status.getDescription());
        }
        return status;
    }

    /**
     * Sets the torque current output of the Kraken hardware component.
     * 
     * @param amps the desired current output in amps
     * @return the status code indicating the success or failure of the operation
     */
    public StatusCode setTorqueCurrentOutput(double amps) {
        StatusCode status = setControl(torqueRequest.withOutput(amps));
        if (status.isError()) {
            System.err.println("Failure to set current output");
            System.err.println("Error Code " + status.value + " On Device " + getDeviceID() + " - " + status.getDescription());
        }
        return status;
    }

    /**
     * Sets the position conversion factor for the Kraken.
     * This factor is used to convert the raw position value to a meaningful unit.
     *
     * @param newFactor the new position conversion factor to be set
     */
    public void setPositionConversionFactor(double newFactor) {
        positionConversionFactor = newFactor;
    }

    /**
     * Sets the velocity conversion factor for the Kraken.
     * This factor is used to convert the desired velocity value to the actual velocity value
     * that is sent to the Kraken motor controller.
     *
     * @param newFactor the new velocity conversion factor to be set
     */
    public void setVelocityConversionFactor(double newFactor) {
        velocityConversionFactor = newFactor;
    }

    /**
     * Applies a parameter to the device configuration using the given configuration application and configuration name.
     * 
     * @param configApplication the supplier that applies the configuration parameter
     * @param configName the name of the configuration parameter
     * @return the status code indicating the result of applying the parameter
     */
    public StatusCode applyParameter(Supplier<StatusCode> configApplication, Supplier<StatusCode> refreshConfig, BooleanSupplier parameterCheck, String configName) {
        return DeviceUtil.applyParameter(configApplication, refreshConfig, parameterCheck, configName, getDeviceID());
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
     * Applies the given signal frequency to the specified status signals for the Kraken device.
     * 
     * @param frequency The frequency at which to apply the signals.
     * @param signals The status signals to apply the frequency to.
     * @return The status code indicating the success or failure of applying the signal frequency.
     */
    public StatusCode applySignalFrequency(double frequency, BaseStatusSignal... signals) {
        return DeviceUtil.applySignalFrequency(frequency, getDeviceID(), signals);
    }

    /**
     * Restores the factory defaults of the TalonFX.
     * 
     * @return The status code indicating the result of the operation.
     */
    public StatusCode restoreFactoryDefaults() {
        return applyParameter(
            () -> configurator.apply(new TalonFXConfiguration(), 1.0),
            "Factory Defaults"
        );
    }

    /**
     * Sets the motor inversion status.
     *
     * @param inverted true to invert the motor output, false otherwise
     * @return the status code indicating the success or failure of the operation
     */
    public StatusCode setMotorInverted(boolean inverted) {
        InvertedValue invertedValue = inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        outputConfigs.Inverted = invertedValue;
        return applyParameter(
            () -> configurator.apply(outputConfigs, 1.0),
            () -> configurator.refresh(outputConfigs, 1.0),
            () -> outputConfigs.Inverted == invertedValue,
            "Motor Output Inverted"
        );
    }

    /**
     * Sets the supply current limit for the Kraken.
     * 
     * It is worth noting that supply current is effectively limited by the 
     * stator current limit, as Supply Current = Duty Cycle * Stator Current,
     * where Duty Cycle never exceeds a magnitude of 1.0.
     * 
     * @param currentLimit the desired current limit in amperes
     * @return the status code indicating the success or failure of the operation
     */
    public StatusCode setSupplyCurrentLimit(double currentLimit) {
        currentLimitConfigs.SupplyCurrentLimit = currentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;

        return applyParameter(
            () -> configurator.apply(currentLimitConfigs, 1.0), 
            () -> configurator.refresh(currentLimitConfigs, 1.0),
            () -> (currentLimitConfigs.SupplyCurrentLimit != 0 ^ currentLimit == 0) 
                && currentLimitConfigs.SupplyCurrentLimitEnable,
            "Supply Current Limit"
        );
    }

    /**
     * Sets the stator current limit for the Kraken.
     * 
     * This current limit will also affect the supply current,
     * as supply current cannot excede the stator current.
     * 
     * @param currentLimit the desired stator current limit
     * @return the status code indicating the success or failure of the operation
     */
    public StatusCode setStatorCurrentLimit(double currentLimit) {
        currentLimitConfigs.StatorCurrentLimit = currentLimit;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        return applyParameter(
            () -> configurator.apply(currentLimitConfigs, 1.0),
            () -> configurator.refresh(currentLimitConfigs, 1.0),
            () -> (currentLimitConfigs.StatorCurrentLimit != 0 ^ currentLimit == 0) 
                && currentLimitConfigs.StatorCurrentLimitEnable,
            "Stator Current Limit"
        );
    }   

    /**
     * Sets the torque current limits for the Kraken.
     * 
     * @param reverseLimit The reverse torque current limit.
     * @param forwardLimit The forward torque current limit.
     * @return The status code indicating the success or failure of the operation.
     */
    public StatusCode setTorqueCurrentLimits(double reverseLimit, double forwardLimit) {
        torqueCurrentConfigs.PeakReverseTorqueCurrent = reverseLimit;
        torqueCurrentConfigs.PeakForwardTorqueCurrent = forwardLimit;
        return applyParameter(
            () -> configurator.apply(torqueCurrentConfigs, 1.0),
            () -> configurator.apply(torqueCurrentConfigs, 1.0),
            () -> (torqueCurrentConfigs.PeakReverseTorqueCurrent != 0 ^ reverseLimit == 0)
                && (torqueCurrentConfigs.PeakForwardTorqueCurrent != 0 ^ forwardLimit == 0),
            "Torque Current Limits"
        );
    }

    /**
     * Sets the closed loop ramp period for the motor controller.
     * This is the time to ramp from 0V to 12V (or 0A to 300A) in seconds
     * 
     * @param seconds the ramp period in seconds
     * @return the status code indicating the success or failure of the operation
     */
    public StatusCode setClosedLoopRampPeriod(double seconds) {
        if (useTorqueControl) {
            closedLoopRampConfigs.TorqueClosedLoopRampPeriod = seconds;
        } else {
            closedLoopRampConfigs.VoltageClosedLoopRampPeriod = seconds;
        }
        return applyParameter(
            () -> configurator.apply(closedLoopRampConfigs, 1.0),
            () -> configurator.refresh(closedLoopRampConfigs, 1.0),
            () -> (useTorqueControl ? 
                closedLoopRampConfigs.TorqueClosedLoopRampPeriod != 0 : 
                closedLoopRampConfigs.VoltageClosedLoopRampPeriod != 0)
                ^ seconds == 0,
            "Closed Loop Ramp Period"
        );
    }
    
    /**
     * Resets the encoder to the specified position.
     * 
     * @param position the desired position to reset the encoder to (with the PCF applied)
     * @return the status code indicating the result of the operation
     */
    public StatusCode resetEncoder(double position) {
        return applyParameter(
            () -> setPosition(position / positionConversionFactor, 1.0),
            "Internal Encoder Reset"
        );
    }

    public StatusCode resetEncoder() {
        return resetEncoder(0);
    }
    
    /**
     * Sets the brake mode for the Kraken hardware.
     * 
     * @param brake true to enable brake mode, false to enable coast mode
     * @return the status code indicating the success or failure of the operation
     */
    public StatusCode setBrakeMode(boolean brake) {
        NeutralModeValue neutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        outputConfigs.NeutralMode = neutralMode;
        return applyParameter(
            () -> configurator.apply(outputConfigs, 1.0),
            () -> configurator.refresh(outputConfigs, 1.0),
            () -> outputConfigs.NeutralMode == neutralMode,
            "Brake Mode"
        );
    }
    
    /**
     * Sets the encoder for the specified CANCoder ID with the given mechanism reduction.
     * 
     * @param canCoderId The ID of the CANCoder.
     * @param mechanismReduction The mechanism reduction ratio.
     * @return The status code indicating the result of the operation.
     */
    public StatusCode setEncoder(int canCoderId, double mechanismReduction) {
        feedbackConfigs.FeedbackRemoteSensorID = canCoderId;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        feedbackConfigs.SensorToMechanismRatio = 1.0;
        feedbackConfigs.RotorToSensorRatio = mechanismReduction;
        return applyParameter(
            () -> configurator.apply(feedbackConfigs, 1.0),
            () -> configurator.refresh(feedbackConfigs, 1.0),
            () -> feedbackConfigs.FeedbackRemoteSensorID == canCoderId 
                && feedbackConfigs.FeedbackSensorSource == FeedbackSensorSourceValue.FusedCANcoder
                && feedbackConfigs.SensorToMechanismRatio == 1.0
                && (feedbackConfigs.RotorToSensorRatio != 0 ^ mechanismReduction == 0),
            "Absolute Encoder Configuration"
        );
    }

    /**
     * Sets the position closed loop wrapping enabled flag.
     * 
     * @param enabled true to enable wrapping, false otherwise
     * @return the status code indicating the success or failure of the operation
     */
    public StatusCode setPositionClosedLoopWrappingEnabled(boolean enabled) {
        closedLoopConfigs.ContinuousWrap = enabled;
        return applyParameter(
            () -> configurator.apply(closedLoopConfigs, 1.0),
            () -> configurator.refresh(closedLoopConfigs, 1.0),
            () -> closedLoopConfigs.ContinuousWrap == enabled,
            "PID Wrapping Enabled"
        );
    }
    
    /**
     * Adds a follower motor to the current motor.
     * 
     * @param motor The motor to be added as a follower.
     * @param invert Specifies whether the follower motor should be inverted or not.
     * @return The status code indicating the success or failure of adding the follower motor.
     */
    public StatusCode addFollower(Kraken motor, boolean invert) {
        StatusCode status = motor.setControl(new Follower(getDeviceID(), invert));
        if (status.isError()) {
            System.err.println("Failure to add follower");
            System.err.println("Error Code " + status.value + " - " + status.getDescription());
        }
        return status;
    }

    /**
     * Adds a follower motor to this Kraken motor.
     * 
     * @param motor the motor to be added as a follower
     * @return the status code indicating the success or failure of the operation
     */
    public StatusCode addFollower(Kraken motor) {
        return addFollower(motor, false);
    }

    /**
     * Obtains position conversion factor of the Kraken.
     * 
     * @return position conversion factor
     */
    public double getPositionConversionFactor() {
        return positionConversionFactor;
    }

    /**
     * Obtains velocity conversion factor of the Kraken.
     * 
     * @return velocity conversion factor
     */
    public double getVelocityConversionFactor() {
        return velocityConversionFactor;
    }

    /**
     * Obtains target position of the Kraken.
     * 
     * @return target position
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Obtains target velocity of the Kraken.
     * 
     * @return target velocity
     */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * Obtains target percent speed of the Kraken.
     * 
     * @return target percent speed
     */
    public double getTargetPercent() {
        return targetPercent;
    }

    /**
     * Obtains the current position of the Kraken as a double.
     * 
     * @return current position as rotations * PCF
     */
    public double getPositionAsDouble() {
        return positionSignal.getValue().in(Rotations) * positionConversionFactor;        
    }

    /**
     * Obtains the current velocity of the Kraken as a double.
     * 
     * @return current velocity as rotations per second * VCF
     */
    public double getVelocityAsDouble() {
        return velocitySignal.getValue().in(RotationsPerSecond) * velocityConversionFactor;
    }

    /**
     * Obtains the current voltage of the Kraken as a double.
     * 
     * @return current voltage in volts
     */
    public double getVoltageAsDouble() {
        return voltageSignal.getValue().in(Volts);
    }

    /**
     * Obtains the current duty cycle output percentage of the Kraken as a double.
     * 
     * @return current percent from -1.0 to 1.0
     */
    public double getPercentAsDouble() {
        return percentSignal.getValue();
    }

    /**
     * Represents current supplied to Kraken as a double.
     * 
     * @return supply of current to Kraken in amps
     */
    public double getSupplyCurrentAsDouble() {
        return supplyCurrentSignal.getValue().in(Amps);
    }

    /**
     * Represents current supplied to the stator of the Kraken as a double.
     * 
     * @return stator current in amps, directly proportional to applied torque
     */
    public double getStatorCurrentAsDouble() {
        return statorCurrentSignal.getValue().in(Amps);
    }

    /**
     * Represents the current creating torque as a double.
     * 
     * @return torque current in amps, directly proportional to applied torque
     */
    public double getTorqueCurrentAsDouble() {
        return torqueCurrentSignal.getValue().in(Amps);
    }

    /**
     * Represents the current motor temperature in celcius as a double.
     * 
     * @return motor temp. in celcius
     */
    public double getTemperatureAsDouble() {
        return temperatureSignal.getValue().in(Celsius);
    }

    /**
     * Refreshes the signals based on the telemetry preference.
     * 
     * @return The status code indicating the success or failure of the signal refresh.
     */
    public StatusCode refreshSignals() {
        return 
            switch(telemetryPreference) {
                case NO_ENCODER -> 
                    BaseStatusSignal.refreshAll(
                        voltageSignal,
                        percentSignal,
                        supplyCurrentSignal,
                        statorCurrentSignal,
                        torqueCurrentSignal,
                        temperatureSignal
                    );
                case SWERVE ->
                    BaseStatusSignal.refreshAll(
                        voltageSignal,
                        positionSignal,
                        velocitySignal,
                        supplyCurrentSignal,
                        torqueCurrentSignal,
                        temperatureSignal
                    );
                default ->
                    BaseStatusSignal.refreshAll(
                        positionSignal,
                        velocitySignal,
                        voltageSignal,
                        percentSignal,
                        supplyCurrentSignal,
                        statorCurrentSignal,
                        torqueCurrentSignal,
                        temperatureSignal
                    );
            };
    }

    /**
     * Adds the Kraken to motor map.
     */
    public void register() {
        KrakenMotorConstants.KRAKEN_MOTOR_MAP.put(getDeviceID(), this);
        if (FieldConstants.IS_SIMULATION) {
            motorSimModel = 
                new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                        useFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1), 
                        0.001, 
                        1), 
                    useFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1));
        }
    }

    /**
     * Updates voltage, position and velocity in sim.
     */
    public void tick() {

        sim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = sim.getMotorVoltage();
  
        motorSimModel.setInputVoltage(motorVoltage);
        motorSimModel.update(0.020);

        sim.setRawRotorPosition(motorSimModel.getAngularPositionRotations());
        sim.setRotorVelocity(Units.radiansToRotations(motorSimModel.getAngularVelocityRadPerSec()));

    }

    /**
     * Applies the specified gains to the given slot.
     * 
     * @param appliedGains the gains to be applied
     * @param slot the slot number to apply the gains to
     * @return the status code indicating the success or failure of the operation
     */
    private StatusCode applyGains(GainConstants appliedGains, int slot) {
        if (slot < 0 || slot > 2) {
            slot = 0;
        }
        gains[slot] = appliedGains;
        slotConfigs.SlotNumber = slot;
        slotConfigs.kP = appliedGains.getP();
        slotConfigs.kI = appliedGains.getI();
        slotConfigs.kD = appliedGains.getD();
        slotConfigs.kS = appliedGains.getS();
        slotConfigs.kV = appliedGains.getV();
        slotConfigs.kG = appliedGains.getG();
        return applyParameter(
            () -> configurator.apply(slotConfigs, 1.0),
            () -> configurator.refresh(slotConfigs, 1.0),
            () -> (slotConfigs.kP != 0 ^ appliedGains.getP() == 0) 
                && (slotConfigs.kI != 0 ^ appliedGains.getI() == 0)
                && (slotConfigs.kD != 0 ^ appliedGains.getD() == 0)
                && (slotConfigs.kS != 0 ^ appliedGains.getS() == 0)
                && (slotConfigs.kV != 0 ^ appliedGains.getV() == 0)
                && (slotConfigs.kG != 0 ^ appliedGains.getG() == 0),
            "Gains"
        );
    }

    /**
     * Sets the gains for a specific slot in the Kraken hardware.
     * 
     * @param P the proportional gain value
     * @param I the integral gain value
     * @param D the derivative gain value
     * @param S the feed forward gain value
     * @param V the velocity gain value
     * @param G the acceleration gain value
     * @param slot the slot number to set the gains for
     * @return the status code indicating the success or failure of the operation
     */
    public StatusCode setGains(double P, double I, double D, double S, double V, double G, int slot)  {
        return applyGains(gains[slot].withGains(P, I, D, S, V, G), slot);
    }

    public StatusCode setGains(double P, double I, double D, double S, double V, double G) {
        return setGains(P, I, D, S, V, G, 0);
    }

    public StatusCode setGains(double P, double I, double D, double S, double V) {
        return setGains(P, I, D, S, V, 0, 0);
    }

    /**
     * Sets the gains for a specific slot and applies them.
     *
     * @param constants the gain constants to set
     * @param slot the slot index to set the gains for
     * @return the status code indicating the success or failure of the operation
     */
    public StatusCode setGains(GainConstants constants, int slot) {
        return applyGains(constants, slot);
    }

    public StatusCode setGains(GainConstants constants) {
        return setGains(constants, 0);
    }

    /**
     * Sets the PID gains for a specific slot and applies them to the motor controller.
     * 
     * @param P the proportional gain value
     * @param I the integral gain value
     * @param D the derivative gain value
     * @param slot the slot number to set the gains for
     * @return the status code indicating the success or failure of the operation
     */
    public StatusCode setPID(double P, double I, double D, int slot) {
        return applyGains(gains[slot].withPID(P, I, D), slot);
    }

    public StatusCode setPID(double P, double I, double D) {
        return setPID(P, I, D, 0);
    }

    /**
     * Sets the proportional gain (P) for a specific slot and applies the updated gains to the Kraken.
     * 
     * @param P the proportional gain value to set
     * @param slot the slot index to set the gain for
     * @return the status code indicating the success or failure of the operation
     */
    public StatusCode setP(double P, int slot) {
        return applyGains(gains[slot].withP(P), slot);
    }

    public StatusCode setP(double P) {
        return setP(P, 0);
    }

    /**
     * Sets the value of the I (integral) gain for the specified slot in the gains array.
     * 
     * @param I    the new value of the I gain
     * @param slot the index of the slot in the gains array
     * @return     the status code indicating the success or failure of the operation
     */
    public StatusCode setI(double I, int slot) {
        return applyGains(gains[slot].withI(I), slot);
    }

    public StatusCode setI(double I) {
        return setI(I, 0);
    }

    /**
     * Sets the value of the D (derivative) gain for the specified slot in the gains array.
     * 
     * @param D    the new value of the D gain
     * @param slot the index of the slot in the gains array
     * @return     the status code indicating the success or failure of the operation
     */
    public StatusCode setD(double D, int slot) {
        return applyGains(gains[slot].withD(D), slot);
    }

    public StatusCode setD(double D) {
        return setD(D, 0);
    }

    /**
     * Sets the value of kS (static friction gain) for a specific slot and applies the gains.
     *
     * @param S    the value of S to set
     * @param slot the slot to apply the gains to
     * @return the status code indicating the result of the operation
     */
    public StatusCode setS(double S, int slot) {
        return applyGains(gains[slot].withS(S), slot);
    }

    public StatusCode setS(double S) {
        return setS(S, 0);
    }

    /**
     * Sets the kV value for the specified slot and applies the gains.
     *
     * @param V    the V value to set
     * @param slot the slot index
     * @return the status code indicating the result of the operation
     */
    public StatusCode setV(double V, int slot) {
        return applyGains(gains[slot].withV(V), slot);
    }

    public StatusCode setV(double V) {
        return setV(V, 0);
    }
    
    /**
     * Sets the value of G (gain) for the specified slot and applies the updated gains to the Kraken.
     *
     * @param G    the new value of G (gain)
     * @param slot the slot number to update the gains for
     * @return the status code indicating the success or failure of the operation
     */
    public StatusCode setG(double G, int slot) {
        return applyGains(gains[slot].withG(G), slot);
    }

    public StatusCode setG(double G) {
        return setG(G, 0);
    }

    /**
     * Gets the proportional gain from the specified slot
     * 
     * @param slot the slot to get the gain from
     * @return the proportional gain
     */
    public double getP(int slot) {
        return gains[slot].getP();
    }

    /**
     * Gets the proportional gain from slot 0
     * 
     * @return the proportional gain
     */
    public double getP() {
        return getP(0);
    }

    /**
     * Gets the integral gain from the specified slot
     * 
     * @param slot the slot to get the gain from
     * @return the integral gain
     */
    public double getI(int slot) {
        return gains[slot].getI();
    }

    /**
     * Gets the integral gain from slot 0
     * 
     * @return the integral gain
     */
    public double getI() {
        return getI(0);
    }

    /**
     * Gets the derivative gain from the specified slot
     * 
     * @param slot the slot to get the gain from
     * @return the derivative gain
     */
    public double getD(int slot) {
        return gains[slot].getD();
    }

    /**
     * Gets the derivative gain from slot 0
     * 
     * @return the derivative gain
     */
    public double getD() {
        return getD(0);
    }

    /**
     * Gets the static feedforward gain from the specified slot
     * 
     * @param slot the slot to get the gain from
     * @return the static feedforward gain
     */
    public double getS(int slot) {
        return gains[slot].getS();
    }

    /**
     * Gets the static feedforward gain from slot 0
     * 
     * @return the static feedforward gain
     */
    public double getS() {
        return getS(0);
    }

    /**
     * Gets the velocity feedforward gain from the specified slot
     * 
     * @param slot the slot to get the gain from
     * @return the velocity feedforward gain
     */
    public double getV(int slot) {
        return gains[slot].getV();
    }

    /**
     * Gets the velocity feedforward gain from slot 0
     * 
     * @return the velocity feedforward gain
     */
    public double getV() {
        return getV(0);
    }

    /**
     * Gets the gravitational feedforward gain from the specified slot
     * 
     * @param slot the slot to get the gain from
     * @return the gravitational feedforward gain
     */
    public double getG(int slot) {
        return gains[slot].getG();
    }

    /**
     * Gets the gravitational feedforward gain from slot 0
     * 
     * @return the gravitational feedforward gain
     */
    public double getG() {
        return getG(0);
    }

}
