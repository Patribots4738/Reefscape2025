package frc.robot.subsystems.superstructure.wrist;

import frc.robot.util.Constants.WristConstants;
import frc.robot.util.hardware.rev.Neo;

public class WristIONeo implements WristIO {

    private final Neo motor;

    public WristIONeo() {
        motor = new Neo(WristConstants.WRIST_CAN_ID, false, false, true);
        configMotor();
    }

    private void configMotor() {
        motor.setPID(WristConstants.WRIST_GAINS);
        motor.setPositionConversionFactor(WristConstants.MOTOR_POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(WristConstants.MOTOR_VELOCITY_CONVERSION_FACTOR / 60.0);
        motor.setSmartCurrentLimit((int) WristConstants.CURRENT_LIMIT);
        setBrakeMode(WristConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.internalPositionRads = motor.getPosition();
        inputs.internalVelocityRadsPerSec = motor.getVelocity();
        inputs.targetPositionRads = motor.getTargetPosition();
        inputs.appliedOutputVolts = motor.getBusVoltage();
        inputs.supplyCurrentAmps = motor.getOutputCurrent();
        inputs.temperatureCelcius = motor.getMotorTemperature();

        inputs.encoderConnected = true;
        inputs.encoderPositionRads = motor.getPosition();
    }

    @Override
    public void setPosition(double position) {
        motor.setTargetPosition(position);
    }

    @Override
    public void runCharacterization(double input) {
        motor.setVoltage(input);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        if (brake) {
            motor.setBrakeMode();
        } else {
            motor.setCoastMode();
        }
    }
    
}
