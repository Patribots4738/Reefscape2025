package frc.robot.subsystems.superstructure.climb;

import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.rev.ThroughBoreEncoder;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.FieldConstants;

public class ClimbIOKraken implements ClimbIO {
    
    private final Kraken leader;
    private final Kraken follower;
    private final ThroughBoreEncoder encoder;

    public ClimbIOKraken() {
        leader = new Kraken(ClimbConstants.CLIMB_LEADER_CAN_ID, true, false);
        follower = new Kraken(ClimbConstants.CLIMB_FOLLOWER_CAN_ID, true, false);
        encoder = new ThroughBoreEncoder(ClimbConstants.CLIMB_ENCODER_DIO_PIN);
        configEncoder();
        configMotors();
    }

    private void configEncoder() {
        encoder.setInverted(ClimbConstants.CLIMB_ENCODER_INVERTED);
        encoder.setPositionOffsetRotations(ClimbConstants.CLIMB_ENCODER_POSITION_OFFSET_ROTATIONS);
        encoder.setPositionConversionFactor(ClimbConstants.ENCODER_POSITION_CONVERSION_FACTOR);
    }

    private void configMotor(Kraken motor) {
        motor.setPositionConversionFactor(ClimbConstants.MOTOR_POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(ClimbConstants.MOTOR_VELOCITY_CONVERSION_FACTOR);
        motor.resetEncoder(encoder.getPosition());
        motor.setGains(ClimbConstants.CLIMB_GAINS);
        motor.setSupplyCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        motor.setStatorCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        motor.setTorqueCurrentLimits(-ClimbConstants.CURRENT_LIMIT, ClimbConstants.CURRENT_LIMIT);
    }

    private void configMotors() {
        configMotor(leader);
        configMotor(follower);
        setBrakeMode(ClimbConstants.BRAKE_MOTOR);
    }

    public void updateInputs(ClimbIOInputs inputs) {
        inputs.leaderMotorConnected = leader.refreshSignals().isOK();
        inputs.leaderPositionRads = leader.getPositionAsDouble();
        inputs.leaderVelocityRadsPerSec = leader.getVelocityAsDouble();
        inputs.leaderTargetPositionRads = leader.getTargetPosition();
        inputs.leaderAppliedOutputVolts = leader.getVoltageAsDouble();
        inputs.leaderSupplyCurrentAmps = leader.getSupplyCurrentAsDouble();
        inputs.leaderStatorCurrentAmps = leader.getStatorCurrentAsDouble();
        inputs.leaderTorqueCurrentAmps = leader.getTorqueCurrentAsDouble();
        inputs.leaderTemperatureCelcius = leader.getTemperatureAsDouble();

        inputs.followerMotorConnected = follower.refreshSignals().isOK();
        inputs.followerPositionRads = follower.getPositionAsDouble();
        inputs.followerVelocityRadsPerSec = follower.getVelocityAsDouble();
        inputs.followerTargetPositionRads = follower.getTargetPosition();
        inputs.followerAppliedOutputVolts = follower.getVoltageAsDouble();
        inputs.followerSupplyCurrentAmps = follower.getSupplyCurrentAsDouble();
        inputs.followerStatorCurrentAmps = follower.getStatorCurrentAsDouble();
        inputs.followerTorqueCurrentAmps = follower.getTorqueCurrentAsDouble();
        inputs.followerTemperatureCelcius = follower.getTemperatureAsDouble();

        if (!FieldConstants.IS_SIMULATION) {
            inputs.encoderConnected = encoder.isConnected();
        }
        inputs.encoderPositionRads = inputs.encoderConnected ? encoder.getPosition() : leader.getPositionAsDouble();
    }

    public void setPosition(double position) {
        leader.setTargetPosition(position);
        follower.setTargetPosition(position);
    }

    public void runCharacterization(double input) {
        leader.setVoltageOutput(input);
        follower.setVoltageOutput(input);
    }

    public void setBrakeMode(boolean brake) {
        leader.setBrakeMode(brake);
        follower.setBrakeMode(brake);
    }

}
