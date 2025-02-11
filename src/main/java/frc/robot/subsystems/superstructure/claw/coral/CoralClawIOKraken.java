package frc.robot.subsystems.superstructure.claw.coral;

import frc.robot.subsystems.superstructure.claw.ClawIO;
import frc.robot.util.Constants.CoralClawConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;
import frc.robot.util.hardware.phoenix.Kraken.TelemetryPreference;

public class CoralClawIOKraken implements ClawIO {
    
    // private final Kraken motor;

    public CoralClawIOKraken() {
        // motor = new Kraken(CoralClawConstants.CAN_ID, true, false, ControlPreference.VOLTAGE);
        configMotor();
    }

    private void configMotor() {
        // motor.setMotorInverted(CoralClawConstants.MOTOR_INVERTED);
        // motor.setTelemetryPreference(TelemetryPreference.NO_ENCODER);
        // motor.setSupplyCurrentLimit(CoralClawConstants.CURRENT_LIMIT);
        // motor.setStatorCurrentLimit(CoralClawConstants.CURRENT_LIMIT);
        setBrakeMode(CoralClawConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        // inputs.motorConnected = motor.refreshSignals().isOK();
        // inputs.percentOutput = motor.getPercentAsDouble();
        // inputs.targetPercentOutput = motor.getTargetPercent();
        // inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        // inputs.supplyCurrentAmps = motor.getSupplyCurrentAsDouble();
        // inputs.statorCurrentAmps = motor.getStatorCurrentAsDouble();
        // inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        // inputs.temperatureCelsius = motor.getTemperatureAsDouble();
    }

    @Override
    public void setNeutral() {
        // motor.setNeutral();
    }

    @Override
    public void setBrakeMode(boolean brake) {
        // motor.setBrakeMode(brake);
    }

    @Override
    public void setPercent(double percent) {
        // motor.setPercentOutput(percent);
    }

    @Override
    public void setVoltage(double volts) {
        // motor.setVoltageOutput(volts);
    }

    @Override
    public void setCurrent(double amps) {
        // motor.setTorqueCurrentOutput(amps);
    }

}
