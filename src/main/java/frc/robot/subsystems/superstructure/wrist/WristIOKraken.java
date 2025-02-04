package frc.robot.subsystems.superstructure.wrist;

import frc.robot.util.Constants.WristConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;

public class WristIOKraken implements WristIO {
    
    private final Kraken motor;

    public WristIOKraken() {
        motor = new Kraken(WristConstants.CAN_ID, true, true, ControlPreference.TORQUE_CURRENT);
        configMotor();
    }

    private void configMotor() {
        motor.setGearRatio(WristConstants.GEAR_RATIO);
        motor.setUnitConversionFactor(WristConstants.POSITION_CONVERSION_FACTOR);
        motor.setSoftLimits(WristConstants.MIN_ANGLE_RADIANS, WristConstants.MAX_ANGLE_RADIANS);
        motor.setMotorInverted(WristConstants.MOTOR_INVERTED);
        motor.resetEncoder(WristConstants.RESET_ANGLE_RADIANS);
        motor.setGains(WristConstants.GAINS.withG(0.0));
        motor.setStatorCurrentLimit(WristConstants.CURRENT_LIMIT);
        motor.setTorqueCurrentLimits(-WristConstants.CURRENT_LIMIT, WristConstants.CURRENT_LIMIT);
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
    public void setPosition(double position, double feedforward) {
        motor.setTargetPosition(position, feedforward);
    }

    @Override
    public void setNeutral() {
        motor.setNeutral();
    }

    @Override
    public void runCharacterization(double input) {
        motor.setTorqueCurrentOutput(input);
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
