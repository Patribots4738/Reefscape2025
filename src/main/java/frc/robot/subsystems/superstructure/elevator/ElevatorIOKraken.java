package frc.robot.subsystems.superstructure.elevator;

import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.Kraken;

public class ElevatorIOKraken implements ElevatorIO {

    private final Kraken leader;
    private final Kraken follower;

    public ElevatorIOKraken() {
        leader = new Kraken(ElevatorConstants.LEADER_CAN_ID, true, false);
        follower = new Kraken(ElevatorConstants.FOLLOWER_CAN_ID, true, false);
        configMotors();
    }

    private void configMotor(Kraken motor) {
        motor.setGearRatio(ElevatorConstants.GEAR_RATIO);
        motor.setUnitConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR);
        motor.setSoftLimits(0.0, ElevatorConstants.MAX_DISPLACEMENT_METERS);
        motor.setMotorInverted(ElevatorConstants.MOTOR_INVERTED);
        motor.setGains(ElevatorConstants.GAINS);
        motor.setSupplyCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
        motor.setStatorCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
    }

    private void configMotors() {
        resetEncoder(0);
        configMotor(leader);
        configMotor(follower);
        setBrakeMode(ElevatorConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leaderMotorConnected = leader.refreshSignals().isOK();
        inputs.leaderPositionMeters = leader.getPositionAsDouble();
        inputs.leaderTargetPositionMeters = leader.getTargetPosition();
        inputs.leaderVelocityMetersPerSecond = leader.getVelocityAsDouble();
        inputs.leaderAppliedOutputVolts = leader.getVoltageAsDouble();
        inputs.leaderSupplyCurrentAmps = leader.getSupplyCurrentAsDouble();
        inputs.leaderStatorCurrentAmps = leader.getStatorCurrentAsDouble();
        inputs.leaderTorqueCurrentAmps = leader.getTorqueCurrentAsDouble();
        inputs.leaderTemperatureCelsius = leader.getTemperatureAsDouble();

        inputs.followerMotorConnected = follower.refreshSignals().isOK();
        inputs.followerPositionMeters = follower.getPositionAsDouble();
        inputs.followerTargetPositionMeters = follower.getTargetPosition();
        inputs.followerVelocityMetersPerSecond = follower.getVelocityAsDouble();
        inputs.followerAppliedOutputVolts = follower.getVoltageAsDouble();
        inputs.followerSupplyCurrentAmps = follower.getSupplyCurrentAsDouble();
        inputs.followerStatorCurrentAmps = follower.getStatorCurrentAsDouble();
        inputs.followerTorqueCurrentAmps = follower.getTorqueCurrentAsDouble();
        inputs.followerTemperatureCelsius = follower.getTemperatureAsDouble();
    }

    @Override
    public void setPosition(double position) {
        leader.setTargetPosition(position);
        follower.setTargetPosition(position);
    }

    @Override
    public void runCharacterization(double input) {
        leader.setVoltageOutput(input);
        follower.setVoltageOutput(input);
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        leader.setBrakeMode(brake);
        follower.setBrakeMode(brake);
    }

    @Override
    public void resetEncoder(double position) {
        leader.resetEncoder(position);
        follower.resetEncoder(position);
    }
    
    @Override
    public void setGains(GainConstants constants) {
        leader.setGains(constants);
        follower.setGains(constants);
    }

}
