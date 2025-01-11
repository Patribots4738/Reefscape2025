package frc.robot.subsystems.superstructure.wrist;

import frc.robot.util.Constants.WristConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.hardware.phoenix.CANCoderCustom;
import frc.robot.util.hardware.phoenix.Kraken;

public class WristIOKraken implements WristIO {
    
    private final Kraken motor;
    private final CANCoderCustom encoder;

    public WristIOKraken() {
        motor = new Kraken(WristConstants.WRIST_CAN_ID, true, false);
        encoder = new CANCoderCustom(WristConstants.WRIST_CANCODER_CAN_ID);
        configEncoder();
        configMotor();
    }

    private void configEncoder() {
        encoder.configureMagnetSensor(false, WristConstants.WRIST_CANCODER_OFFSET);
        encoder.setPositionConversionFactor(WristConstants.POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(WristConstants.VELOCITY_CONVERSION_FACTOR);
    }

    private void configMotor() {
        if (!FieldConstants.IS_SIMULATION) {
            motor.setEncoder(encoder.getDeviceID(), WristConstants.GEAR_RATIO);
        }
        motor.setGains(WristConstants.WRIST_GAINS);
        motor.setPositionConversionFactor(WristConstants.POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(WristConstants.VELOCITY_CONVERSION_FACTOR);
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
        inputs.temperatureCelcius = motor.getTemperatureAsDouble();

        if (!FieldConstants.IS_SIMULATION) {
            inputs.encoderConnected = encoder.refreshSignals().isOK();
        }
        inputs.encoderPositionRads = inputs.encoderConnected ? encoder.getPositionAsDouble() : motor.getPositionAsDouble();
        inputs.encoderAbsPositionRads = inputs.encoderConnected ? encoder.getAbsolutePositionAsDouble() : motor.getPositionAsDouble();
        inputs.encoderVelocityRadsPerSec = inputs.encoderConnected ? encoder.getVelocityAsDouble() : motor.getVelocityAsDouble();
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
