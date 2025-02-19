package frc.robot.subsystems.superstructure.climb;

import frc.robot.util.hardware.rev.Neo;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.custom.GainConstants;

public class ClimbIONeo implements ClimbIO {
    
    private final Neo motor;

    public ClimbIONeo() {
        motor = new Neo(ClimbConstants.CAN_ID, false, false, true);
        configMotors();
    }


    private void configMotors() {
        motor.setOutputInverted(ClimbConstants.MOTOR_INVERTED);
        motor.setPositionConversionFactor(ClimbConstants.POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(ClimbConstants.VELOCITY_CONVERSION_FACTOR / 60.0);
        motor.setPID(ClimbConstants.SLOW_GAINS, 0);
        motor.setPID(ClimbConstants.FAST_GAINS, 1);
        motor.setSmartCurrentLimit((int) ClimbConstants.CURRENT_LIMIT);
        setBrakeMode(ClimbConstants.BRAKE_MOTOR);
    }

    public void updateInputs(ClimbIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.positionRads = motor.getPosition();
        inputs.velocityRadsPerSec = motor.getVelocity();
        inputs.targetPositionRads = motor.getTargetPosition();
        inputs.appliedOutputVolts = motor.getBusVoltage();
        inputs.supplyCurrentAmps = motor.getOutputCurrent();
        inputs.temperatureCelcius = motor.getMotorTemperature();
    }

    public void setPosition(double position) {
        motor.setTargetPosition(position);
    }

    public void runCharacterization(double input) {
        motor.setVoltage(input);
    }

    public void setBrakeMode(boolean brake) {
        if (brake) {
            motor.setBrakeMode();
        } else {
            motor.setCoastMode();
        }
    }

    @Override
    public void setGains(GainConstants constants, int slot) {
        motor.setPID(constants, slot);
    }


}
