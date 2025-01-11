package frc.robot.subsystems.superstructure.claw;

import frc.robot.util.Constants.ClawConstants;
import frc.robot.util.hardware.rev.Neo;
import frc.robot.util.hardware.rev.SafeSpark.TelemetryPreference;

public class ClawIONeo implements ClawIO {
    
    private final Neo motor;

    public ClawIONeo() {
        motor = new Neo(ClawConstants.CLAW_CAN_ID, false);
        configMotor();
    }

    private void configMotor() {
        motor.setTelemetryPreference(TelemetryPreference.NO_ENCODER);
        motor.setSmartCurrentLimit((int) ClawConstants.CURRENT_LIMIT);
        setBrakeMode(ClawConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.percentOutput = motor.getAppliedOutput();
        inputs.targetPercentOutput = motor.getTargetPercent();
        inputs.appliedOutputVolts = motor.getBusVoltage();
        inputs.supplyCurrentAmps = motor.getOutputCurrent();
        inputs.temperatureCelcius = motor.getMotorTemperature();
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
