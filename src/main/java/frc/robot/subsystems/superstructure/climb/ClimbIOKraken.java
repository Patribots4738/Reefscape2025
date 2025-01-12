package frc.robot.subsystems.superstructure.climb;

import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.Constants.ClimbConstants;

public class ClimbIOKraken {
    
    private final Kraken leader;
    private final Kraken follower;

    public ClimbIOKraken() {
        leader = new Kraken(ClimbConstants.CLIMB_LEADER_CAN_ID, true, false);
        follower = new Kraken(ClimbConstants.CLIMB_FOLLOWER_CAN_ID, true, false);
        configMotors();
    }

    private void configMotors() {
        configMotor(leader);
        configMotor(follower);
    }

    private void configMotor(Kraken motor) {
        motor.setGains(ClimbConstants.CLIMB_GAINS);
        motor.setPositionConversionFactor(ClimbConstants.POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(ClimbConstants.VELOCITY_CONVERSION_FACTOR);
        motor.setSupplyCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        motor.setStatorCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        motor.setTorqueCurrentLimits(-ClimbConstants.CURRENT_LIMIT, ClimbConstants.CURRENT_LIMIT);
    }

}
