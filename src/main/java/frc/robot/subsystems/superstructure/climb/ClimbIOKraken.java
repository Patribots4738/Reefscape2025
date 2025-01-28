package frc.robot.subsystems.superstructure.climb;

import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.Constants.ClimbConstants;

public class ClimbIOKraken implements ClimbIO {
    
    private final Kraken leader;
    private final Kraken follower;

    public ClimbIOKraken() {
        leader = new Kraken(ClimbConstants.CLIMB_LEADER_CAN_ID, true, false);
        follower = new Kraken(ClimbConstants.CLIMB_FOLLOWER_CAN_ID, true, false);
        configMotors();
    }

    private void configMotor(Kraken motor) {
        motor.setMotorInverted(true);
        motor.setPositionConversionFactor(ClimbConstants.POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(ClimbConstants.VELOCITY_CONVERSION_FACTOR);
        motor.resetEncoder(0);
        motor.setGains(ClimbConstants.CLIMB_GAINS);
        motor.setSupplyCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        motor.setStatorCurrentLimit(ClimbConstants.CURRENT_LIMIT);
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
