package frc.robot.subsystems.superstructure.climb;

import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.custom.GainConstants;

public class ClimbIOKraken implements ClimbIO {
    
    private final Kraken motor;

    public ClimbIOKraken() {
        motor = new Kraken(ClimbConstants.CAN_ID, true, false, ControlPreference.TORQUE_CURRENT);
        
        configMotor();
    }

    private void configMotor() {
        motor.setGearRatio(ClimbConstants.GEAR_RATIO);
        motor.setUnitConversionFactor(ClimbConstants.POSITION_CONVERSION_FACTOR);
        motor.setSoftLimits(ClimbConstants.MIN_ANGLE_RADIANS, ClimbConstants.MAX_ANGLE_RADIANS);
        motor.setMotorInverted(ClimbConstants.MOTOR_INVERTED);
        motor.resetEncoder(0);
        motor.setGains(ClimbConstants.SLOW_GAINS, 0);
        motor.setGains(ClimbConstants.FAST_GAINS, 1);
        motor.setStatorCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        motor.setTorqueCurrentLimits(-ClimbConstants.CURRENT_LIMIT, ClimbConstants.CURRENT_LIMIT);
        configureProfile(ClimbConstants.VELOCITY, ClimbConstants.ACCELERATION, ClimbConstants.JERK);
        setBrakeMode(ClimbConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.motorConnected = motor.refreshSignals().isOK();
        inputs.positionRads = motor.getPositionAsDouble();
        inputs.velocityRadsPerSec = motor.getVelocityAsDouble();
        inputs.targetPositionRads = motor.getTargetPosition();
        inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        inputs.temperatureCelcius = motor.getTemperatureAsDouble();
    }

    @Override
    public void setNeutral() {
        motor.setNeutral();
    }

    @Override
    public void setPosition(double position, int slot) {
        motor.setTargetPosition(position, slot);
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
    public void resetEncoder(double position) {
        motor.resetEncoder(position);
    }

    @Override
    public void setGains(GainConstants constants, int slot) {
        motor.setGains(constants, slot);
    }

    @Override
    public void configureProfile(double velocity, double acceleration, double jerk) {
        motor.configureMotionMagic(velocity, acceleration, jerk);
    }

}
