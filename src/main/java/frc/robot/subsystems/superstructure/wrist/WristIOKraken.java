package frc.robot.subsystems.superstructure.wrist;

import frc.robot.util.Constants.WristConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.Kraken;

public class WristIOKraken implements WristIO {
    
    private final Kraken motor;

    public WristIOKraken() {
        motor = new Kraken(WristConstants.CAN_ID, true, false);
        configMotor();
    }

    private void configMotor() {
        motor.setPositionConversionFactor(WristConstants.POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(WristConstants.VELOCITY_CONVERSION_FACTOR);
        motor.setMotorInverted(WristConstants.MOTOR_INVERTED);
        motor.resetEncoder(WristConstants.RESET_ANGLE_RADIANS);
        motor.setGains(WristConstants.GAINS);
        motor.setSupplyCurrentLimit(WristConstants.CURRENT_LIMIT);
        motor.setStatorCurrentLimit(WristConstants.CURRENT_LIMIT);
        setBrakeMode(WristConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.motorConnected = motor.refreshSignals().isOK();
        inputs.positionRads = motor.getPositionAsDouble();
        inputs.velocityRadsPerSec = motor.getVelocityAsDouble();
        inputs.targetPositionRads = motor.getTargetPosition();
        inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrentAsDouble();
        inputs.statorCurrentAmps = motor.getStatorCurrentAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        inputs.temperatureCelsius = motor.getTemperatureAsDouble();
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

    @Override
    public void setGains(GainConstants constants) {
        motor.setGains(constants);
    }

}
