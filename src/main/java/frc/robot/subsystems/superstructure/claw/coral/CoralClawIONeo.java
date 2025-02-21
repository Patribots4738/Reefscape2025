package frc.robot.subsystems.superstructure.claw.coral;

import frc.robot.subsystems.superstructure.claw.ClawIO;
import frc.robot.util.Constants.CoralClawConstants;
import frc.robot.util.hardware.rev.Neo;

public class CoralClawIONeo implements ClawIO {
    
    private final Neo motor;

    public CoralClawIONeo() {
        motor = new Neo(CoralClawConstants.CAN_ID, false);
        configMotor();
    }

    private void configMotor() {
        motor.setOutputInverted(CoralClawConstants.MOTOR_INVERTED);
        motor.setSmartCurrentLimit((int) CoralClawConstants.CURRENT_LIMIT);
        setBrakeMode(CoralClawConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.percentOutput = motor.getAppliedOutput();
        inputs.targetPercentOutput = motor.getTargetPercent();
        inputs.appliedOutputVolts = motor.getBusVoltage();
        inputs.torqueCurrentAmps = motor.getOutputCurrent();
        inputs.temperatureCelsius = motor.getMotorTemperature();
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
    public void setPercent(double percent) {
        motor.setTargetPercent(percent);
    }

}
