package frc.robot.subsystems.superstructure.wrist;

import frc.robot.util.Constants.WristConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.rev.ThroughBoreEncoder;

public class WristIOKraken implements WristIO {
    
    private final Kraken motor;
    private final ThroughBoreEncoder encoder;

    public WristIOKraken() {
        motor = new Kraken(WristConstants.WRIST_CAN_ID, true, false);
        encoder = new ThroughBoreEncoder(WristConstants.WRIST_ENCODER_DIO_PIN);
        configEncoder();
        configMotor();
    }

    private void configEncoder() {
        encoder.setInverted(WristConstants.WRIST_ENCODER_INVERTED);
        encoder.setPositionOffsetRotations(WristConstants.WRIST_ENCODER_POSITION_OFFSET_ROTATIONS);
        encoder.setPositionConversionFactor(WristConstants.ENCODER_POSITION_CONVERSION_FACTOR);
    }

    private void configMotor() {
        motor.setPositionConversionFactor(WristConstants.MOTOR_POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(WristConstants.MOTOR_VELOCITY_CONVERSION_FACTOR);
        motor.resetEncoder(encoder.getPosition());
        motor.setGains(WristConstants.WRIST_GAINS);
        motor.setSupplyCurrentLimit(WristConstants.CURRENT_LIMIT);
        motor.setStatorCurrentLimit(WristConstants.CURRENT_LIMIT);
        motor.setTorqueCurrentLimits(-WristConstants.CURRENT_LIMIT, WristConstants.CURRENT_LIMIT);
        setBrakeMode(WristConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.motorConnected = motor.refreshSignals().isOK();
        inputs.internalPositionRads = motor.getPositionAsDouble();
        inputs.internalVelocityRadsPerSec = motor.getVelocityAsDouble();
        inputs.targetPositionRads = motor.getTargetPosition();
        inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrentAsDouble();
        inputs.statorCurrentAmps = motor.getStatorCurrentAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        inputs.temperatureCelsius = motor.getTemperatureAsDouble();

        if (!FieldConstants.IS_SIMULATION) {
            inputs.encoderConnected = encoder.isConnected();
        }
        inputs.encoderPositionRads = inputs.encoderConnected ? encoder.getPosition() : motor.getPositionAsDouble();
    }

    @Override
    public void setPosition(double position) {
        motor.setTargetPosition(position);
    }

    @Override
    public void runCharacterization(double input) {
        motor.setVoltageOutput(input);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        motor.setBrakeMode(brake);
    }

}
