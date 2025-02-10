package frc.robot.subsystems.superstructure.climb;

import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.custom.GainConstants;

public class ClimbIOKraken implements ClimbIO {
    
    private final Kraken leader;
    // private final Kraken follower;

    public ClimbIOKraken() {
        leader = new Kraken(ClimbConstants.LEADER_CAN_ID, true, false, ControlPreference.TORQUE_CURRENT);
        // follower = new Kraken(ClimbConstants.FOLLOWER_CAN_ID, true, false, ControlPreference.MM_TORQUE_CURRENT);
        
        configMotors();
    }

    private void configMotor(Kraken motor) {
        motor.setGearRatio(ClimbConstants.GEAR_RATIO);
        motor.setUnitConversionFactor(ClimbConstants.POSITION_CONVERSION_FACTOR);
        motor.setSoftLimits(ClimbConstants.MIN_ANGLE_RADIANS, ClimbConstants.MAX_ANGLE_RADIANS);
        motor.setMotorInverted(ClimbConstants.MOTOR_INVERTED);
        motor.resetEncoder(0);
        motor.setGains(ClimbConstants.GAINS);
        motor.setStatorCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        motor.setTorqueCurrentLimits(-ClimbConstants.CURRENT_LIMIT, ClimbConstants.CURRENT_LIMIT);
    }

    private void configMotors() {
        configMotor(leader);
        // configMotor(follower);
        configureProfile(ClimbConstants.VELOCITY, ClimbConstants.ACCELERATION, ClimbConstants.JERK);
        setBrakeMode(ClimbConstants.BRAKE_MOTOR);
    }

    @Override
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

        // inputs.followerMotorConnected = follower.refreshSignals().isOK();
        // inputs.followerPositionRads = follower.getPositionAsDouble();
        // inputs.followerVelocityRadsPerSec = follower.getVelocityAsDouble();
        // inputs.followerTargetPositionRads = follower.getTargetPosition();
        // inputs.followerAppliedOutputVolts = follower.getVoltageAsDouble();
        // inputs.followerSupplyCurrentAmps = follower.getSupplyCurrentAsDouble();
        // inputs.followerStatorCurrentAmps = follower.getStatorCurrentAsDouble();
        // inputs.followerTorqueCurrentAmps = follower.getTorqueCurrentAsDouble();
        // inputs.followerTemperatureCelcius = follower.getTemperatureAsDouble();
    }

    @Override
    public void setNeutral() {
        leader.setNeutral();
        // follower.setFollowing(leader);
    }

    @Override
    public void setPosition(double position) {
        leader.setTargetPosition(position);
        // follower.setFollowing(leader);
    }

    @Override
    public void runCharacterization(double input) {
        leader.setTorqueCurrentOutput(input);
        // follower.setFollowing(leader);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        leader.setBrakeMode(brake);
        // follower.setBrakeMode(brake);
    }

    @Override
    public void setGains(GainConstants constants) {
        leader.setGains(constants);
        // follower.setGains(constants);
    }

    @Override
    public void configureProfile(double velocity, double acceleration, double jerk) {
        leader.configureMotionMagic(velocity, acceleration, jerk);
        // follower.configureMotionMagic(velocity, acceleration, jerk);
    }

}
