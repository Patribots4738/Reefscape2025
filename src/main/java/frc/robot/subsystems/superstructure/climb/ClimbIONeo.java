package frc.robot.subsystems.superstructure.climb;

import frc.robot.util.hardware.rev.Neo;
import frc.robot.util.Constants.ClimbConstants;

public class ClimbIONeo implements ClimbIO {
    
    private final Neo leader;
    private final Neo follower;

    public ClimbIONeo() {
        leader = new Neo(ClimbConstants.CLIMB_LEADER_CAN_ID, false, false, true);
        follower = new Neo(ClimbConstants.CLIMB_FOLLOWER_CAN_ID, false, false, false);
        configMotors();
    }

    private void configMotor(Neo motor) {
        motor.setPositionConversionFactor(ClimbConstants.MOTOR_POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(ClimbConstants.MOTOR_VELOCITY_CONVERSION_FACTOR / 60.0);
        motor.setPID(ClimbConstants.CLIMB_GAINS);
        motor.setSmartCurrentLimit((int) ClimbConstants.CURRENT_LIMIT);
    }

    private void configMotors() {
        configMotor(leader);
        configMotor(follower);
        setBrakeMode(ClimbConstants.BRAKE_MOTOR);
    }

    public void updateInputs(ClimbIOInputs inputs) {
        inputs.leaderMotorConnected = true;
        inputs.leaderPositionRads = leader.getPosition();
        inputs.leaderVelocityRadsPerSec = leader.getVelocity();
        inputs.leaderTargetPositionRads = leader.getTargetPosition();
        inputs.leaderAppliedOutputVolts = leader.getBusVoltage();
        inputs.leaderSupplyCurrentAmps = leader.getOutputCurrent();
        inputs.leaderTemperatureCelcius = leader.getMotorTemperature();

        inputs.followerMotorConnected = true;
        inputs.followerPositionRads = follower.getPosition();
        inputs.followerVelocityRadsPerSec = follower.getVelocity();
        inputs.followerTargetPositionRads = follower.getTargetPosition();
        inputs.followerAppliedOutputVolts = follower.getBusVoltage();
        inputs.followerSupplyCurrentAmps = follower.getOutputCurrent();
        inputs.followerTemperatureCelcius = follower.getMotorTemperature();

        inputs.encoderConnected = true;
        inputs.encoderPositionRads = leader.getPosition();
    }

    public void setPosition(double position) {
        leader.setTargetPosition(position);
        follower.setTargetPosition(position);
    }

    public void runCharacterization(double input) {
        leader.setVoltage(input);
        follower.setVoltage(input);
    }

    public void setBrakeMode(boolean brake) {
        if (brake) {
            leader.setBrakeMode();
            follower.setBrakeMode();
        } else {
            leader.setCoastMode();
            follower.setBrakeMode();
        }
    }

}
