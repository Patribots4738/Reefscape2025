package frc.robot.subsystems.superstructure.climb;

import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.custom.GainConstants;

public class ClimbIOKraken implements ClimbIO {
    
    // private final Kraken motor;

    public ClimbIOKraken() {
        // motor = new Kraken(ClimbConstants.CAN_ID, true, false, ControlPreference.TORQUE_CURRENT);
        
        configMotor();
    }

    private void configMotor() {
        // motor.setGearRatio(ClimbConstants.GEAR_RATIO);
        // motor.setUnitConversionFactor(ClimbConstants.POSITION_CONVERSION_FACTOR);
        // motor.setSoftLimits(ClimbConstants.MIN_ANGLE_RADIANS, ClimbConstants.MAX_ANGLE_RADIANS);
        // motor.setMotorInverted(ClimbConstants.MOTOR_INVERTED);
        // motor.resetEncoder(0);
        // motor.setGains(ClimbConstants.GAINS);
        // motor.setStatorCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        // motor.setTorqueCurrentLimits(-ClimbConstants.CURRENT_LIMIT, ClimbConstants.CURRENT_LIMIT);
        configureProfile(ClimbConstants.VELOCITY, ClimbConstants.ACCELERATION, ClimbConstants.JERK);
        setBrakeMode(ClimbConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        // inputs.motorConnected = motor.refreshSignals().isOK();
        // inputs.positionRads = motor.getPositionAsDouble();
        // inputs.velocityRadsPerSec = motor.getVelocityAsDouble();
        // inputs.targetPositionRads = motor.getTargetPosition();
        // inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        // inputs.supplyCurrentAmps = motor.getSupplyCurrentAsDouble();
        // inputs.statorCurrentAmps = motor.getStatorCurrentAsDouble();
        // inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        // inputs.temperatureCelcius = motor.getTemperatureAsDouble();
    }

    @Override
    public void setNeutral() {
        // motor.setNeutral();
    }

    @Override
    public void setPosition(double position) {
        // motor.setTargetPosition(position);
    }

    @Override
    public void runCharacterization(double input) {
        // motor.setTorqueCurrentOutput(input);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        // motor.setBrakeMode(brake);
    }

    @Override
    public void resetEncoder(double position) {
        // motor.resetEncoder(position);
    }

    @Override
    public void setGains(GainConstants constants) {
        // motor.setGains(constants);
    }

    @Override
    public void configureProfile(double velocity, double acceleration, double jerk) {
        // motor.configureMotionMagic(velocity, acceleration, jerk);
    }

}
