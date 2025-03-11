package frc.robot.subsystems.superstructure.claw.algae;

import frc.robot.subsystems.superstructure.claw.ClawIO;
import frc.robot.util.Constants.AlgaeClawConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;

public class AlgaeClawIOKraken implements ClawIO {
    
    private final Kraken motor;

    public AlgaeClawIOKraken() {
        motor = new Kraken(AlgaeClawConstants.CAN_ID, true, true, ControlPreference.VOLTAGE);
        configMotor();
    }

    private void configMotor() {
        motor.setMotorInverted(AlgaeClawConstants.MOTOR_INVERTED);
        motor.setSupplyCurrentLimit(AlgaeClawConstants.CURRENT_LIMIT);
        motor.setStatorCurrentLimit(AlgaeClawConstants.CURRENT_LIMIT);
        setBrakeMode(AlgaeClawConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        inputs.motorConnected = motor.refreshSignals().isOK();
        inputs.percentOutput = motor.getPercentAsDouble();
        inputs.targetPercentOutput = motor.getTargetPercent();
        inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        inputs.temperatureCelsius = motor.getTemperatureAsDouble();
        inputs.velocityRotationsPerSec = motor.getVelocityAsDouble();
    }

    @Override
    public void setNeutral() {
        motor.setNeutral();
    }

    @Override
    public void setBrakeMode(boolean brake) {
        motor.setBrakeMode(brake);
    }

    @Override
    public void setPercent(double percent) {
        motor.setPercentOutput(percent);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltageOutput(volts);
    }

    @Override
    public void setCurrent(double amps) {
        motor.setTorqueCurrentOutput(amps);
    }

}
