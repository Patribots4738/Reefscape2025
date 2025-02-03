package frc.robot.subsystems.superstructure.claw;

import frc.robot.util.Constants.ClawConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;
import frc.robot.util.hardware.phoenix.Kraken.TelemetryPreference;

public class ClawIOKraken implements ClawIO {
    
    private final Kraken motor;

    public ClawIOKraken() {
        motor = new Kraken(ClawConstants.CAN_ID, true, false, ControlPreference.VOLTAGE);
        configMotor();
    }

    private void configMotor() {
        motor.setMotorInverted(ClawConstants.MOTOR_INVERTED);
        motor.setTelemetryPreference(TelemetryPreference.NO_ENCODER);
        motor.setSupplyCurrentLimit(ClawConstants.CURRENT_LIMIT);
        motor.setStatorCurrentLimit(ClawConstants.CURRENT_LIMIT);
        setBrakeMode(ClawConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        inputs.motorConnected = motor.refreshSignals().isOK();
        inputs.percentOutput = motor.getPercentAsDouble();
        inputs.targetPercentOutput = motor.getTargetPercent();
        inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrentAsDouble();
        inputs.statorCurrentAmps = motor.getStatorCurrentAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        inputs.temperatureCelsius = motor.getTemperatureAsDouble();
    }

    @Override
    public void setBrakeMode(boolean brake) {
        motor.setBrakeMode(brake);
    }

    @Override
    public void setPercent(double percent) {
        motor.setPercentOutput(percent);
    }

}
