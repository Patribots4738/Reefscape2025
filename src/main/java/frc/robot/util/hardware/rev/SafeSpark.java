// Primarily referenced from https://github.com/lasarobotics/PurpleLib/blob/master/src/main/java/org/lasarobotics/hardware/revrobotics/Spark.java
package frc.robot.util.hardware.rev;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SoftLimitConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkFlexConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.GeneralHardwareConstants;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.custom.GainConstants;

public class SafeSpark extends SparkBase {

    protected final boolean isSparkFlex;
    protected final int canID;
    protected final boolean useAbsoluteEncoder;
    protected final SparkClosedLoopController pidController = getClosedLoopController();
    protected RelativeEncoder relativeEncoder;
    protected SparkAbsoluteEncoder absoluteEncoder;
    protected final SparkBaseConfig config;
    protected final SparkBaseConfigAccessor accessor;

    private final int MAX_ATTEMPTS = (GeneralHardwareConstants.SAFE_HARDWARE_MODE) ? 20 : 2;
    private final int CAN_TIMEOUT_MS = 50;
    private final int SPARK_MAX_MEASUREMENT_PERIOD = 16;
    private final int SPARK_FLEX_MEASUREMENT_PERIOD = 32;
    private final int SPARK_MAX_AVERAGE_DEPTH = 2;
    private final int SPARK_FLEX_AVERAGE_DEPTH = 8;
    private final double BURN_FLASH_WAIT_TIME = (NeoMotorConstants.SAFE_SPARK_MODE) ? 0.1 : 0.05;
    private final double APPLY_PARAMETER_WAIT_TIME = (NeoMotorConstants.SAFE_SPARK_MODE) ? 0.05 : 0;

    public SafeSpark(int canID, boolean useAbsoluteEncoder, MotorType motorType, boolean isSparkFlex) {
        super(canID, motorType, isSparkFlex ? SparkModel.SparkFlex : SparkModel.SparkMax);

        this.canID = canID;
        this.useAbsoluteEncoder = useAbsoluteEncoder;
        this.isSparkFlex = isSparkFlex;
        config = isSparkFlex ? new SparkFlexConfig() : new SparkMaxConfig();
        accessor = isSparkFlex ? new SparkFlexConfigAccessor(sparkHandle) : new SparkMaxConfigAccessor(sparkHandle);

        // Set the motor to factory default settings
        // we do this so we can hot-swap sparks without needing to reconfigure them
        // this requires the code to configure the sparks after construction
        restoreFactoryDefaults();
        // Add a delay to let the spark reset
        // If a parameter set fails, this will add more time 
        // to minimize any bus traffic.
        // Default is 20ms
        setCANTimeout(CAN_TIMEOUT_MS);
        
        resetStatusFrame(StatusFrame.APPLIED_FAULTS);
        resetStatusFrame(StatusFrame.VELO_TEMP_VOLTAGE_CURRENT);
        resetStatusFrame(StatusFrame.ENCODER_POSITION);
        resetStatusFrame(StatusFrame.ALL_ANALOG_ENCODER);
        resetStatusFrame(StatusFrame.ALL_ALTERNATE_ENCODER);
        resetStatusFrame(StatusFrame.ABSOLUTE_ENCODER_POS);
        resetStatusFrame(StatusFrame.ABSOLUTE_ENCODER_VELO);

        if (useAbsoluteEncoder) {
            setFeedbackDevice(FeedbackSensor.kAbsoluteEncoder);
        }
        if (motorType == SparkBase.MotorType.kBrushless) {
            fixMeasurementPeriod();
            fixAverageDepth();
        }
    }

    public int getCANID() {
        return canID;
    }

    /**
     * Attempt to apply parameter and check if specified parameter is set correctly
     * 
     * @param parameterSetter        Method to set desired parameter
     * @param parameterCheckSupplier Method to check for parameter in question
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError applyParameter(
        Supplier<REVLibError> parameterSetter, 
        BooleanSupplier parameterCheckSupplier, 
        String errorMessage)
    {
        if (FieldConstants.IS_SIMULATION)
            return parameterSetter.get();

        REVLibError status = REVLibError.kError;
        for (int i = 0; i < MAX_ATTEMPTS; i++) {
            status = parameterSetter.get();
            if (parameterCheckSupplier.getAsBoolean() && status == REVLibError.kOk)
                break;
            Timer.delay(APPLY_PARAMETER_WAIT_TIME);
        }

        checkStatus(status, errorMessage);
        return status;
    }

    public REVLibError applyParameter(BooleanSupplier parameterCheckSupplier, String errorMessage) {
        return applyParameter(() -> configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters), parameterCheckSupplier, errorMessage);
    }

    /**
     * Restore motor controller parameters to factory defaults until the next
     * controller reboot
     * 
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError restoreFactoryDefaults() {
        REVLibError status = applyParameter(
            () -> configure(config.apply(isSparkFlex ? new SparkFlexConfig() : new SparkMaxConfig()), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
            () -> true,
            "Restore factory defaults failure!");
        Timer.delay(BURN_FLASH_WAIT_TIME);
        return status;
    }

    /**
     * Check status and print error message if necessary
     * 
     * @param status       Status to check
     * @param errorMessage Error message to print
     */
    public void checkStatus(REVLibError status, String errorMessage) {
        if (status != REVLibError.kOk) {
            System.err.println(canID + " (" + NeoMotorConstants.CAN_ID_MAP.get(canID) + ") " + errorMessage + " - "
                    + status.toString());
        }
        if (getFaults().sensor) {
            String message = "\nSensor fault detected on motor " +
                canID + " (" + NeoMotorConstants.CAN_ID_MAP.get(canID) + ")" +
                ". Power cycle the robot to fix.";
            for (int i = 0; i < 5; i++) {
                System.err.println(message);
                Timer.delay(.1);
            }
        }
    }

    @Override
    public RelativeEncoder getEncoder() {
        if (relativeEncoder == null) {
            relativeEncoder = super.getEncoder();
        }
        return relativeEncoder;
    }

    public RelativeEncoder getRelativeEncoder() {
        return this.getEncoder();
    }

    @Override
    public SparkAbsoluteEncoder getAbsoluteEncoder() {
        if (absoluteEncoder == null) {
            absoluteEncoder = super.getAbsoluteEncoder();
        }
        return absoluteEncoder;
    }

    /**
     * Set encoder velocity measurement period to {@value Spark#MEASUREMENT_PERIOD}
     * milliseconds
     * 
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError fixMeasurementPeriod() {
        final int MEASUREMENT_PERIOD = isSparkFlex ? SPARK_FLEX_MEASUREMENT_PERIOD : SPARK_MAX_MEASUREMENT_PERIOD;
        config.encoder.uvwMeasurementPeriod(MEASUREMENT_PERIOD).quadratureMeasurementPeriod(MEASUREMENT_PERIOD);
        REVLibError status = applyParameter(
                () -> (accessor.encoder.getUvwMeasurementPeriod() == MEASUREMENT_PERIOD && accessor.encoder.getQuadratureMeasurementPeriod() == MEASUREMENT_PERIOD),
                "Set encoder measurement period failure!");
        return status;
    }

    /**
     * Set encoder velocity average depth to {@value Spark#AVERAGE_DEPTH} samples
     * 
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError fixAverageDepth() {
        final int AVERAGE_DEPTH = isSparkFlex ? SPARK_FLEX_AVERAGE_DEPTH : SPARK_MAX_AVERAGE_DEPTH;
        config.encoder.uvwAverageDepth(AVERAGE_DEPTH).quadratureAverageDepth(AVERAGE_DEPTH);
        REVLibError status = applyParameter(
                () -> accessor.encoder.getUvwAverageDepth() == AVERAGE_DEPTH && accessor.encoder.getQuadratureAverageDepth() == AVERAGE_DEPTH,
                "Set encoder average depth failure!");
        return status;
    }

    /**
     * Invert the motor.
     * 
     * @param inverted to invert or not to invert, that is the question
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError invertMotor(boolean inverted) {
        return applyParameter(
                () -> {
                    throwIfClosed();
                    if (useAbsoluteEncoder) {
                        config.absoluteEncoder.inverted(inverted);
                    } else {    
                        config.inverted(inverted);
                    }
                    return configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                },
                () -> (useAbsoluteEncoder ? accessor.absoluteEncoder.getInverted() == inverted : accessor.getInverted() == inverted),
                "Set inverted failure!");
    }

    /**
     * Invert the motor
     * 
     */
    @Override
    public void setInverted(boolean inverted) {
        invertMotor(inverted);
    }
    
    /**
     * Set a Spark to follow another Spark
     * 
     * @param leader Spark to follow
     * @param invert Set slave to output opposite of the master
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError follow(SafeSpark leader, boolean invert) {
        config.follow(leader, invert);
        REVLibError status = applyParameter(
            () -> (super.isFollower() && accessor.getFollowerModeInverted() == invert),
            "Set motor master failure!");
        return status;
    }

    /**
     * Set the conversion factor for position of the encoder. Multiplied by the
     * native output units to
     * give you position.
     * 
     * @param sensor Sensor to set conversion factor for
     * @param factor The conversion factor to multiply the native units by
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setPositionConversionFactor(double factor) {
        REVLibError status;

        if (useAbsoluteEncoder) {
            config.absoluteEncoder.positionConversionFactor(factor);
        } else {
            config.encoder.positionConversionFactor(factor);
        }

        status = 
            applyParameter(
                () -> getPositionConversionFactor() == factor, 
                "Set position conversion factor failure!");
        
        return status;
    }

    public double getPositionConversionFactor() {
        if (useAbsoluteEncoder) {
            return accessor.absoluteEncoder.getPositionConversionFactor();
        } else {
            return accessor.encoder.getPositionConversionFactor();
        }
    }

    /**
     * Set the conversion factor for velocity of the encoder. Multiplied by the
     * native output units to
     * give you velocity.
     * 
     * @param sensor Sensor to set conversion factor for
     * @param factor The conversion factor to multiply the native units by
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setVelocityConversionFactor(double factor) {
        REVLibError status;

        if (useAbsoluteEncoder) {
            config.absoluteEncoder.velocityConversionFactor(factor);
        } else {
            config.encoder.velocityConversionFactor(factor);
        }

        status = applyParameter(
            () -> getVelocityConversionFactor() == factor, 
            "Set velocity conversion factor failure!");
        
        return status;
    }

    public REVLibError setSoftLimit(double min, double max) {
        REVLibError status;

        config.softLimit
            .forwardSoftLimit(max)
            .reverseSoftLimit(min)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        SoftLimitConfigAccessor softLimits = accessor.softLimit;

        BooleanSupplier parameterCheckSupplier = () -> 
            softLimits.getForwardSoftLimit() == max &&
            softLimits.getReverseSoftLimit() == min &&
            softLimits.getForwardSoftLimitEnabled() && 
            softLimits.getReverseSoftLimitEnabled();

        status = applyParameter(parameterCheckSupplier, "Set soft limits failure!");
        return status;
    }

    public double getVelocityConversionFactor() {
        if (useAbsoluteEncoder) {
            return accessor.absoluteEncoder.getVelocityConversionFactor();
        } else {
            return accessor.encoder.getVelocityConversionFactor();
        }
    }

    /**
     * Get the position of the encoder
     * This will go through the positionConversionFactor if there is one
     * @return The position of the encoder
     */
    public double getPosition() {
        if (useAbsoluteEncoder && !FieldConstants.IS_SIMULATION) {
            return getAbsoluteEncoder().getPosition();
        } else {
            return getRelativeEncoder().getPosition();
        }
    }

    /**
     * RESET THE RELATIVE ENCODER TO BE THE SET POSITION
     */
    public void setPosition(double position) {
        getRelativeEncoder().setPosition(position);
    }

    /**
     * Get the velocity of the encoder, in RPM
     * This will go through the velocityConversionFactor if there is one
     * @return The velocity of the encoder, in RPM
     */
    public double getVelocity() {
        if (useAbsoluteEncoder) {
            return getAbsoluteEncoder().getVelocity();
        } else {
            return getRelativeEncoder().getVelocity();
        }
    }

    private ClosedLoopSlot getSlotFromInt(int slot) {
        ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;
        switch (slot) {
            case 3:
                closedLoopSlot = ClosedLoopSlot.kSlot3;
                break;
            case 2:
                closedLoopSlot = ClosedLoopSlot.kSlot2;
                break;
            case 1:
                closedLoopSlot = ClosedLoopSlot.kSlot1;
            default:
                break;
        }
        return closedLoopSlot;
    }

    /**
     * Set proportional gain for PIDF controller on Spark
     * 
     * @param value Value to set
     * @param slot Slot to set
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setP(double value, int slot) {
        ClosedLoopSlot gainSlot = getSlotFromInt(slot);
        config.closedLoop.p(value, gainSlot);
        REVLibError status = applyParameter(
            () -> accessor.closedLoop.getP(gainSlot) == value,  
            "Set kP failure!");
        return status;
    }

    public REVLibError setP(double value) {
        return setP(value, 0);
    }

    /**
     * Set integral gain for PIDF controller on Spark
     * 
     * @param value Value to set
     * @param slot Slot to set
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setI(double value, int slot) {
        ClosedLoopSlot gainSlot = getSlotFromInt(slot);
        config.closedLoop.i(value, gainSlot);
        REVLibError status = applyParameter(
            () -> accessor.closedLoop.getI(gainSlot) == value,
            "Set kI failure!");
        return status;
    }

    public REVLibError setI(double value) {
        return setI(value, 0);
    }

    /**
     * Set derivative gain for PIDF controller on Spark
     * 
     * @param value Value to set
     * @param slot Slot to set
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setD(double value, int slot) {
        ClosedLoopSlot gainSlot = getSlotFromInt(slot);
        config.closedLoop.d(value, gainSlot);
        REVLibError status = applyParameter(
            () -> accessor.closedLoop.getD(gainSlot) == value,
            "Set kD failure!");
        return status;
    }

    public REVLibError setD(double value) {
        return setD(value, 0);
    }

    /**
     * Set feed-forward gain for PIDF controller on Spark
     * 
     * @param value Value to set
     * @param slot Slot to set
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setFF(double value, int slot) {
        ClosedLoopSlot gainSlot = getSlotFromInt(slot);
        config.closedLoop.velocityFF(value, gainSlot);
        REVLibError status = applyParameter(
            () -> accessor.closedLoop.getFF(gainSlot) == value,
            "Set kF failure!");
        return status;
    }

    public REVLibError setFF(double value) {
        return setFF(value, 0);
    }

    /**
     * Set integral zone range for PIDF controller on Spark
     * <p>
     * This value specifies the range the |error| must be within for the integral
     * constant to take effect
     * 
     * @param value Value to set
     * @param slot Slot to set
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setIZone(double value, int slot) {
        ClosedLoopSlot gainSlot = getSlotFromInt(slot);
        config.closedLoop.iZone(value, gainSlot);
        REVLibError status = applyParameter(
            () -> accessor.closedLoop.getIZone(gainSlot) == value,
            "Set Izone failure!");
        return status;
    }

    public REVLibError setIZone(double value) {
        return setIZone(value, 0);
    }

    
    /**
     * Gets the proportional gain constant for PID controller.
     * @return The proportional gain constant for PID controller.
     */
    public double getP() {
        return accessor.closedLoop.getP();
    }

    /**
     * Gets the integral gain constant for PID controller.
     * @return The integral gain constant for PID controller.
     */
    public double getI() {
        return accessor.closedLoop.getI();
    }

    /**
     * Gets the derivative gain constant for PID controller.
     * @return The derivative gain constant for PID controller.
     */
    public double getD() {
        return accessor.closedLoop.getD();
    }

    public GainConstants getPID() {
        return new GainConstants(getP(), getI(), getD());
    }

    /**
     * Gets the I-Zone constant for PID controller.
     * The I-Zone is the zone at which the integral
     * will be applied and "enabled"
     *
     * Example: 
     *   an I-Zone of 0.25
     *   will make the integral apply 
     *   for when the absolute value of
     *   | desired - current | is 0.25 or less.
     * 
     * Think of this like the final stretch of PID
     * 
     * @return The I-Zone constant for PID control.
     */
    public double getIZone() {
        return accessor.closedLoop.getIZone();
    }

    /**
     * Gets the feedforward gain constant for PID controller.
     * @return The feedforward gain constant for PID controller.
     */
    public double getFF() {
        return accessor.closedLoop.getFF();
    }

    
    /**
     * Set the min/maximum output for the PIDF controller on Spark
     * 
     * @param min Minimum output value
     * @param max Maximum output value
     * @param slot Slot to set
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setOutputRange(double min, double max, int slot) {
        ClosedLoopSlot gainSlot = getSlotFromInt(slot);
        config.closedLoop.outputRange(min, max, gainSlot);
        ClosedLoopConfigAccessor closedLoop = accessor.closedLoop;
        REVLibError status = applyParameter(
            () -> closedLoop.getMinOutput(gainSlot) == min && closedLoop.getMaxOutput() == max,
            "Set output range failure!");
        return status;
    }

    /**
     * Enable PID wrapping for closed loop position control
     * 
     * @param minInput Value of min input for position
     * @param maxInput Value of max input for position
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError enablePIDWrapping(double minInput, double maxInput) {
        config.closedLoop
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(minInput, maxInput);

        ClosedLoopConfigAccessor closedLoop = accessor.closedLoop;

        BooleanSupplier parameterCheckSupplier = () -> 
                closedLoop.getPositionWrappingEnabled() &&
                closedLoop.getPositionWrappingMinInput() == minInput &&
                closedLoop.getPositionWrappingMaxInput() == maxInput;

        return applyParameter(
            parameterCheckSupplier, 
            "Enable position PID wrapping failure!");
    }

    public boolean getPIDWrappingEnabled() {
        return accessor.closedLoop.getPositionWrappingEnabled();
    }

    /**
     * Set the motor feedback device to the PIDController
     * 
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setFeedbackDevice(FeedbackSensor sensor) {
        config.closedLoop.feedbackSensor(sensor);
        return applyParameter(
            () -> true,
            "Feedback device failure!");
    }

    /**
     * Set the PIDF controller reference for the Spark
     * 
     * @param value        The value to set the reference to
     * @param controlType  The control type to use
     * @param slot         The slot to set
     * @param arbFF        Arbitrary feed forward value
     * @param arbFFUnits   Units for the arbitrary feed forward value
     */
    public void setPIDReference(double value, ControlType controlType, int slot, double arbitraryFeedForward, ArbFFUnits arbFFUnits) {
        pidController.setReference(value, controlType, getSlotFromInt(slot), arbitraryFeedForward, arbFFUnits);
    }

    /**
     * Sets the idle mode setting for the Spark
     * 
     * @param mode Idle mode (coast or brake).
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setIdleMode(IdleMode mode) {
        config.idleMode(mode);
        REVLibError status = applyParameter(
            () -> accessor.getIdleMode() == mode,
            "Set idle mode failure!");
        return status;
    }

    /**
     * Sets the brake mode for the Spark MAX motor controller.
     * 
     * @return The REVLibError indicating the success or failure of the operation.
     */
    public REVLibError setBrakeMode() {
        return this.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Sets the motor controller to coast mode.
     * 
     * @return The REVLibError indicating the success or failure of the operation.
     */
    public REVLibError setCoastMode() {
        return this.setIdleMode(IdleMode.kCoast);
    }

    /**
     * Sets the current limit in Amps.
     *
     * <p>
     * The motor controller will reduce the controller voltage output to avoid
     * surpassing this
     * limit. This limit is enabled by default and used for brushless only. This
     * limit is highly
     * recommended when using the NEO brushless motor.
     *
     * <p>
     * The NEO Brushless Motor has a low internal resistance, which can mean large
     * current spikes
     * that could be enough to cause damage to the motor and controller. This
     * current limit provides a
     * smarter strategy to deal with high current draws and keep the motor and
     * controller operating in
     * a safe region.
     *
     * @param limit The current limit in Amps.
     */
    public REVLibError setSmartCurrentLimit(int limit) {
        config.smartCurrentLimit(limit);
        return applyParameter(
            () -> accessor.getSmartCurrentLimit() == limit,
            "Set current limit failure!");
    }

    /**
     * Change a periodic status frame period of the motor controller.
     * Rev docs:
     * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
     * 
     * You can increase this number to ignore updates more / alliviate can bus traffic
     * or decrease this number to get more frequent updates
     * 
     * @param frame  The frame to change
     * @param period The period to set in milliseconds
     * @return A REVLibError indicating the result of the operation
     */ 
    public REVLibError changeStatusFrame(StatusFrame frame, int period) {
        frame.applyFramePeriod(config, period);
        return configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Resets the status frame of the NEO motor controller to its default period.
     * Rev docs:
     * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
     * 
     * @param frame the status frame to reset
     * @return the REVLibError indicating the result of the operation
     */
    public REVLibError resetStatusFrame(StatusFrame frame) {
        return changeStatusFrame(frame, frame.getDefaultPeriodms());
    }

    /**
     * Represents the status frames for the Neo class.
     * Rev docs:
     * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
     */
    public enum StatusFrame {
        APPLIED_FAULTS(10),
        VELO_TEMP_VOLTAGE_CURRENT(20),
        ENCODER_POSITION(20),
        ALL_ANALOG_ENCODER(50),
        ALL_ALTERNATE_ENCODER(20),
        ABSOLUTE_ENCODER_POS(200),
        ABSOLUTE_ENCODER_VELO(200);

        private final int defaultPeriodms;

        /**
         * Constructs a StatusFrame with the specified frame and default period.
         * 
         * @param frame         The periodic frame.
         * @param defaultPeriod The default period in milliseconds.
         */
        StatusFrame(int defaultPeriod) {
            this.defaultPeriodms = defaultPeriod;
        }

        /**
         * Gets the default period in milliseconds for this StatusFrame.
         * 
         * @return The default period in milliseconds.
         */
        public int getDefaultPeriodms() {
            return defaultPeriodms;
        }

        public void applyFramePeriod(SparkBaseConfig config, int period) {
            SignalsConfig signals = config.signals;
            switch (this) {
                case APPLIED_FAULTS:
                    signals.faultsPeriodMs(period);
                    break;
                case VELO_TEMP_VOLTAGE_CURRENT:
                    signals.primaryEncoderVelocityPeriodMs(period);
                    signals.motorTemperaturePeriodMs(period);
                    signals.appliedOutputPeriodMs(period);
                    signals.outputCurrentPeriodMs(period);
                    break;
                case ENCODER_POSITION:
                    signals.primaryEncoderPositionPeriodMs(period);
                    break;
                case ALL_ANALOG_ENCODER:
                    signals.analogPositionPeriodMs(period);
                    signals.analogVelocityPeriodMs(period);
                    signals.analogVoltagePeriodMs(period);
                    break;
                case ALL_ALTERNATE_ENCODER:
                    signals.externalOrAltEncoderPosition(period);
                    signals.externalOrAltEncoderVelocity(period);
                    break;
                case ABSOLUTE_ENCODER_POS:
                    signals.absoluteEncoderPositionPeriodMs(period);
                    break;
                case ABSOLUTE_ENCODER_VELO:
                    signals.absoluteEncoderVelocityPeriodMs(period);
                    break;
            }
        }

    }

    public enum TelemetryPreference {
        DEFAULT,
        ONLY_ABSOLUTE_ENCODER,
        ONLY_RELATIVE_ENCODER,
        NO_TELEMETRY,
        NO_ENCODER
    }

    /**
     * Set the telemetry preference of the Neo
     * This will disable the telemetry status frames
     * which is found at
     * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
     * 
     * @param type the enum to represent the telemetry preference
     *             this will tell the motor to only send
     *             that type of telemetry
     */
    public void setTelemetryPreference(TelemetryPreference type) {
        int minDelay = NeoMotorConstants.FAST_PERIODIC_STATUS_TIME_MS;
        int maxDelay = NeoMotorConstants.MAX_PERIODIC_STATUS_TIME_MS;

        // No matter what preference, we don't use analog or external encoders.
        changeStatusFrame(StatusFrame.ALL_ALTERNATE_ENCODER, maxDelay);
        changeStatusFrame(StatusFrame.ALL_ANALOG_ENCODER, maxDelay);

        switch (type) {
            // Disable all telemetry that is unrelated to the encoder
            case NO_ENCODER:
                changeStatusFrame(StatusFrame.ENCODER_POSITION, maxDelay);
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_VELO, maxDelay);
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_POS, maxDelay);
                break;
            // Disable all telemetry that is unrelated to absolute encoders
            case ONLY_ABSOLUTE_ENCODER:
                changeStatusFrame(StatusFrame.ENCODER_POSITION, maxDelay);
                changeStatusFrame(StatusFrame.VELO_TEMP_VOLTAGE_CURRENT, maxDelay);
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_POS, minDelay);
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_VELO, minDelay);
                break;
            // Disable all telemetry that is unrelated to the relative encoder
            case ONLY_RELATIVE_ENCODER:
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_VELO, maxDelay);
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_POS, maxDelay);
                changeStatusFrame(StatusFrame.ENCODER_POSITION, minDelay);
                changeStatusFrame(StatusFrame.VELO_TEMP_VOLTAGE_CURRENT, minDelay);
                break;
            // Disable everything
            case NO_TELEMETRY:
                changeStatusFrame(StatusFrame.VELO_TEMP_VOLTAGE_CURRENT, maxDelay);
                changeStatusFrame(StatusFrame.ENCODER_POSITION, maxDelay);
                changeStatusFrame(StatusFrame.ALL_ANALOG_ENCODER, maxDelay);
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_VELO, maxDelay);
                break;

            case DEFAULT:
            default:
                break;
        }
        
    }

}
