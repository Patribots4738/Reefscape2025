package frc.robot.subsystems.superstructure.wrist;

import frc.robot.util.Constants.WristConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.rev.Neo;

public class WristIONeo implements WristIO {

    private final Neo motor;

    public WristIONeo() {
        motor = new Neo(WristConstants.WRIST_CAN_ID, false, false, true);
        configMotor();
    }

    private void configMotor() {
        motor.setOutputInverted(WristConstants.INVERT_MOTOR);
        motor.setPID(WristConstants.WRIST_LOGGED_GAINS);
        motor.setPositionConversionFactor(WristConstants.POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(WristConstants.VELOCITY_CONVERSION_FACTOR / 60.0);
        motor.setSmartCurrentLimit((int) WristConstants.CURRENT_LIMIT);
        setBrakeMode(WristConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.positionRads = motor.getPosition();
        inputs.velocityRadsPerSec = motor.getVelocity();
        inputs.targetPositionRads = motor.getTargetPosition();
        inputs.appliedOutputVolts = motor.getBusVoltage();
        inputs.supplyCurrentAmps = motor.getOutputCurrent();
        inputs.temperatureCelsius = motor.getMotorTemperature();
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

    @Override
    public void setGains(GainConstants constants) {
        motor.setPID(WristConstants.wristLogged.get());
    }

    
}
