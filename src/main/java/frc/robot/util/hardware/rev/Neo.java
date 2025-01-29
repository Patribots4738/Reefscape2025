// Developed in tandem with Reza from 1622

package frc.robot.util.hardware.rev;

import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.custom.GainConstants;
/*
 * Some of this is adapted from 3005's 2022 Code
 * Original source published at https://github.com/FRC3005/Rapid-React-2022-Public/tree/d499655448ed592c85f9cfbbd78336d8841f46e2
 */
public class Neo extends SafeSpark {

    private ControlLoopType controlType = ControlLoopType.PERCENT;
    
    private double targetPosition = 0;
    private double targetVelocity = 0;
    private double targetPercent = 0;

    /**
     * Creates a new Neo motor with the default settings.
     * This assumes no inversion and is relative encoder only
     * 
     * @param id          CANID of the Spark the Neo is connected to.
     * @param isSparkFlex Whether the Spark is a SparkMax or SparkMax Flex
     */
    public Neo(int id, boolean isSparkFlex) {
        this(id, isSparkFlex, false, false);
    }

    /**
     * Creates a new Neo motor with the default settings.
     * This only assumes no absolute encoder
     * 
     * @param id          CANID of the Spark the Neo is connected to.
     * @param isSparkFlex Whether the Spark is a SparkMax or SparkMax Flex
     * @param inverted    Whether the motor is reversed or not.
     */
    public Neo(int id, boolean isSparkFlex, boolean inverted) {
        this(id, isSparkFlex, inverted, false);
    }

    /**
     * Creates a new Neo motor using custom settings
     * 
     * @param id                CANID of the Spark the Neo is connected to.
     * @param isSparkFlex       Whether the Spark is a SparkMax or SparkMax Flex
     * @param inverted          Whether the motor is reversed or not.
     * @param useAbsoluteEncoder Whether the motor uses an absolute encoder or not.
     */
    public Neo(int id, boolean isSparkFlex, boolean inverted, boolean useAbsoluteEncoder) {
        super(id, useAbsoluteEncoder, SparkLowLevel.MotorType.kBrushless, isSparkFlex);
        
        setInverted(inverted);
        
        // Turn off alternate and analog encoders
        // we never use them
        setTelemetryPreference(TelemetryPreference.DEFAULT);
        if (useAbsoluteEncoder) {
            setTelemetryPreference(TelemetryPreference.ONLY_ABSOLUTE_ENCODER);
        } else {
            setTelemetryPreference(TelemetryPreference.ONLY_RELATIVE_ENCODER);
        }
        
        setBrakeMode();
        register();
    }

    /**
     * Sets the target position for the Neo motor
     * with an optional arbitrary feedforward value.
     * 
     * @param position             The target position for the motor.
     * @param arbitraryFeedForward An optional arbitrary feedforward value.
     */
    public void setTargetPosition(double position, double arbitraryFeedForward) {
        setTargetPosition(position, arbitraryFeedForward, 0);
    }

    /**
     * Sets the target position for the Neo motor
     * with an optional arbitrary feedforward value.
     * 
     * @param position The target position for the motor.
     * @param slot     The PID slot for the reference
     */
    public void setTargetPosition(double position, int slot) {
        setTargetPosition(position, 0, slot);
    }

    /**
     * Sets the target position for the Neo motor.
     * 
     * @param position the desired target position
     */
    public void setTargetPosition(double position) {
        setTargetPosition(position, 0, 0);
    }
    
    /**
     * Sets the target position for the Neo motor controller.
     * 
     * @param position             The desired position in encoder units.
     * @param arbitraryFeedForward The arbitrary feedforward value to be applied.
     * @param slot                 The PID slot to use for control.
     */
    public void setTargetPosition(double position, double arbitraryFeedForward, int slot) {
        if (!FieldConstants.IS_SIMULATION) {
            setPIDReference(position, ControlType.kPosition, slot, arbitraryFeedForward, ArbFFUnits.kVoltage);
        }
        targetPosition = position;
        controlType = ControlLoopType.POSITION;
    }

    /**
     * Sets the target percent for the Neo motor.
     * 
     * @param percent the target percent to set
     */
    public void setTargetPercent(double percent) {
        setTargetPercent(percent, 0);
    }

    /**
     * Sets the target percent of speed output for the Neo.
     * 
     * @param percent Percent of Neo output speed.
     * @param slot The PID slot.
     */
    public void setTargetPercent(double percent, int slot) {
        setTargetPercent(percent, 0, slot);
    }

    /**
     * Sets the target percent of speed output for the Neo.
     * 
     * @param percent              Percent of Neo output speed.
     * @param arbitraryFeedForward The FF for the reference
     */
    public void setTargetPercent(double percent, double arbitraryFeedForward) {
        setTargetPercent(percent, arbitraryFeedForward, 0);
    }

    /**
     * Sets the target percent output for the Neo motor controller.
     * 
     * @param percent              The desired percent output (-1.0 to 1.0).
     * @param arbitraryFeedForward The arbitrary feedforward value.
     * @param slot                 The PID slot to use.
     */
    public void setTargetPercent(double percent, double arbitraryFeedForward, int slot) {
        if (percent == 0) {
            setVoltage(0);
        } else {
            setPIDReference(percent, ControlType.kDutyCycle, slot, arbitraryFeedForward, ArbFFUnits.kPercentOut);
        }
        targetPercent = percent;
        controlType = ControlLoopType.PERCENT;
    }

    /**
     * Sets the target velocity for the Neo.
     * @param velocity Velocity to set the Neo to in rotations per minute.
     */
    public void setTargetVelocity(double velocity) {
        setTargetVelocity(velocity, 0);
    }

    /**
     * Sets the target velocity for the Neo.
     * 
     * @param velocity             Velocity to set the Neo to in rotations per
     *                             minute.
     * @param arbitraryFeedForward The FF for the reference.
     */
    public void setTargetVelocity(double velocity, double arbitraryFeedForward) {
        setTargetVelocity(velocity, arbitraryFeedForward, 0);
    }
    
    /**
     * Sets the target velocity for the Neo motor.
     * 
     * @param velocity the target velocity to set
     * @param slot     the slot to set the target velocity for
     */
    public void setTargetVelocity(double velocity, int slot) {
        setTargetVelocity(velocity, 0, slot);
    }

    /**
     * Sets the target velocity for the Neo.
     * 
     * @param velocity             Velocity to set the Neo to in rotations per
     *                             minute.
     * @param arbitraryFeedForward Arbitrary feed forward to add to the motor
     *                             output.
     */
    public void setTargetVelocity(double velocity, double arbitraryFeedForward, int slot) {
        if (velocity == 0) {
            setVoltage(0);
        } else {
            setPIDReference(velocity, ControlType.kVelocity, slot, arbitraryFeedForward, ArbFFUnits.kVoltage);
        }
        targetVelocity = velocity;
        controlType = ControlLoopType.VELOCITY;
    }
    
    /**
     * Sets the output of the Neo motor controller based on a percentage value.
     * Positive values are counter clockwise if there is no reversedMultiplier
     * 
     * @param percent The percentage value to set the motor output to.
     */
    public void set(double percent) {
        targetPercent = percent;
        super.set(percent);
        controlType = ControlLoopType.PERCENT;
    }

    /**
     * Updates the position and velocity of the Neo motor.
     * If caching is enabled, it caches the position and velocity values.
     * If the simulation is active and the control type is position, 
     * it simulates movement on the motor
     */
    public void tick() {
        if ((FieldConstants.IS_SIMULATION) && controlType == ControlLoopType.POSITION) {
            double error = (targetPosition - getPosition());
            if (getPIDWrappingEnabled()) {
                error = MathUtil.angleModulus(error);
            }
            setVoltage(getP() * error);
        }
    }

    /**
     * Registers the Neo motor instance.
     * Adds the motor to the list of registered motors
     * and also adds the motor to the REVPhysicsSim instance
     * if in simulation.
     */
    public void register() {
        NeoMotorConstants.NEO_MOTOR_MAP.put(canID, this);
        
        if (FieldConstants.IS_SIMULATION) {
            DCMotor motor = isSparkFlex ? DCMotor.getNeoVortex(1) : DCMotor.getNeo550(1);
            NeoPhysicsSim.getInstance().addNeo(this, motor);  
        }
    }

    /**
     * Gets the position of the Neo in rotations.
     * @return The position of the Neo in rotations relative to the last 0 position.
     */
    public double getPosition() {
        double pos = super.getPosition();
                
        if (FieldConstants.IS_SIMULATION && controlType == ControlLoopType.VELOCITY) {
            pos /= super.getVelocityConversionFactor();
        }

        return pos;
    }

    /**
     * Gets the velocity of the Neo in rotations per minute.
     * @return The instantaneous velocity of the Neo in rotations per minute.
     */
    public double getVelocity() {
        return super.getVelocity(); 
    }

    /**
     * Sets the position of the Neo encoder.
     * 
     * @param position the desired position to set
     */
    public void resetEncoder(double position) {
        super.setPosition(position);
    }

    /**
     * Set the relative encoder's position to 0
     */
    public void resetEncoder() {
        super.setPosition(0);
    }

    /**
     * Gets the target position of the Neo in rotations.
     * @return The target position of the Neo in rotations.
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Gets the target velocity of the Neo in rotations per minute.
     * @return The target velocity of the Neo in rotations per minute.
     */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * Gets the target percentage of the Neo in... well, %
     * @return The target percent speed of the Neo
     */
    public double getTargetPercent() {
        return targetPercent;
    }

    /**
     * Adds a follower to the Neo motor.
     * 
     * @param follower the Neo motor to be added as a follower
     */
    public void addFollower(Neo follower) {
        addFollower(follower, false);
    }

    /**
     * Adds a follower to the Neo motor.
     * 
     * @param follower The Neo motor to be added as a follower.
     * @param invert   Specifies whether the follower should be inverted or not.
     */
    public void addFollower(Neo follower, boolean invert) {
        follower.follow(this, invert);
    }

    /**
     * Sets the PID values and output limits for the Neo motor controller.
     * 
     * @param P          the proportional gain
     * @param I          the integral gain
     * @param D          the derivative gain
     * @param minOutput  the minimum output value
     * @param maxOutput  the maximum output value
     */
    public void setPID(double P, double I, double D, double minOutput, double maxOutput) {
        setPID(new GainConstants(P, I, D, minOutput, maxOutput));
    }

    /**
     * Sets the PID constants for the Neo motor controller.
     * 
     * @param constants the PID constants to set
     */
    public void setPID(GainConstants constants) {
        setPID(constants, 0);
    }

    /**
     * Sets the PID constants for the Neo motor controller.
     * 
     * @param constants the PID constants to set
     * @param slotID    the slot ID of the PID controller
     */
    public void setPID(GainConstants constants, int slot) {
        setPID(
            constants.getP(), 
            constants.getI(), 
            constants.getD(), 
            constants.getFF(),
            constants.getIZone(), 
            constants.getMinOutput(), 
            constants.getMaxOutput(), 
            slot
        );
    }

    /**
     * Sets the PID values and output range for the PID controller.
     * 
     * @param P          the proportional gain
     * @param I          the integral gain
     * @param D          the derivative gain
     * @param minOutput  the minimum output value
     * @param maxOutput  the maximum output value
     * @param slotID     the slot ID of the PID controller
     */
    public void setPID(double P, double I, double D, double FF, double iZone, double minOutput, double maxOutput, int slotID) {
        super.setP(P, slotID);
        super.setI(I, slotID);
        super.setD(D, slotID);
        super.setFF(FF, slotID);
        super.setIZone(MathUtil.clamp(iZone, 0, 999), slotID);
        super.setOutputRange(minOutput, maxOutput, slotID);
    }

    /**
     * Enum representing the control loop types for the Neo motor.
     * The control loop types include POSITION, VELOCITY, and PERCENT.
     */
    public enum ControlLoopType {
        POSITION,
        VELOCITY,
        PERCENT;
    }

}