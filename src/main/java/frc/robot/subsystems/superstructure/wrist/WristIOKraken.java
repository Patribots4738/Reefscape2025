package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.WristConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;

public class WristIOKraken implements WristIO {
    
    private final Kraken motor;
    private final DutyCycleEncoder encoder;
    private boolean encoderUpdated = false;

    public WristIOKraken() {
        motor = new Kraken(WristConstants.CAN_ID, true, false, !FieldConstants.IS_SIMULATION ? ControlPreference.MM_TORQUE_CURRENT : ControlPreference.TORQUE_CURRENT);
        encoder = new DutyCycleEncoder(WristConstants.ENCODER_DIO_PIN, WristConstants.ENCODER_POSITION_CONVERSION_FACTOR, WristConstants.ENCODER_POSITION_OFFSET_ROTATIONS);
        configMotor();
    }

    private void configMotor() {
        motor.setGearRatio(WristConstants.GEAR_RATIO);
        motor.setUnitConversionFactor(WristConstants.POSITION_CONVERSION_FACTOR);
        motor.setSoftLimits(WristConstants.MIN_ANGLE_RADIANS, WristConstants.MAX_ANGLE_RADIANS);
        motor.setMotorInverted(WristConstants.MOTOR_INVERTED);
        motor.resetEncoder(encoder.get());
        motor.setGains(WristConstants.GAINS);
        motor.setStatorCurrentLimit(WristConstants.CURRENT_LIMIT);
        motor.setTorqueCurrentLimits(-WristConstants.CURRENT_LIMIT, WristConstants.CURRENT_LIMIT);
        configureProfile(WristConstants.VELOCITY, WristConstants.ACCELERATION, WristConstants.JERK);
        setBrakeMode(WristConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.encoderConnected = encoder.isConnected();
        if (inputs.encoderConnected) {
            inputs.encoderAbsPositonRads = encoder.get();
            if (!encoderUpdated) {
                motor.resetEncoder(inputs.encoderAbsPositonRads > WristConstants.MAX_ANGLE_RADIANS ? inputs.encoderAbsPositonRads - 2 * Math.PI : inputs.encoderAbsPositonRads);
                encoderUpdated = true;
            }
        } else {
            inputs.encoderAbsPositonRads = inputs.internalPositionRads;
        }


        inputs.motorConnected = motor.refreshSignals().isOK();
        inputs.internalPositionRads = motor.getPositionAsDouble();
        inputs.internalVelocityRadsPerSec = motor.getVelocityAsDouble();
        inputs.targetPositionRads = motor.getTargetPosition();
        inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        inputs.temperatureCelsius = motor.getTemperatureAsDouble();
    }

    @Override
    public void setPosition(double position, double feedforward) {
        motor.setTargetPosition(position, feedforward);
    }

    @Override
    public void setNeutral() {
        motor.setNeutral();
    }

    @Override
    public void runCharacterization(double input) {
        motor.setTorqueCurrentOutput(input);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        motor.setBrakeMode(brake);
    }

    @Override
    public void setGains(GainConstants constants) {
        motor.setGains(constants.withG(0.0));
    }

    @Override
    public void configureProfile(double velocity, double acceleration, double jerk) {
        motor.configureMotionMagic(velocity, acceleration, jerk);
    }

}
