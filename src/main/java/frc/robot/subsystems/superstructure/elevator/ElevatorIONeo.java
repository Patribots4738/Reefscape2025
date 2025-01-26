package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.Constants.WristConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.rev.Neo;

public class ElevatorIONeo implements ElevatorIO {

    private final Neo leader;
    private final Neo follower;

    public ElevatorIONeo() {
        leader = new Neo(ElevatorConstants.ELEVATOR_LEADER_CAN_ID, false);
        follower = new Neo(ElevatorConstants.ELEVATOR_FOLLOWER_CAN_ID, false);
        configMotors();
    }

    private void configMotor(Neo motor) {
        motor.setPID(ElevatorConstants.ELEVATOR_LOGGED_GAINS);
        motor.setPositionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(ElevatorConstants.VELOCITY_CONVERSION_FACTOR / 60.0);
        motor.setSmartCurrentLimit((int) ElevatorConstants.CURRENT_LIMIT);

        
    }

    private void configMotors() {
        resetEncoder(0);
        configMotor(leader);
        configMotor(follower);
        setBrakeMode(ElevatorConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leaderMotorConnected = true;
        inputs.leaderPositionMeters = leader.getPosition();
        inputs.leaderTargetPositionMeters = leader.getTargetPosition();
        inputs.leaderVelocityMetersPerSecond = leader.getVelocity();
        inputs.leaderAppliedOutputVolts = leader.getBusVoltage();
        inputs.leaderSupplyCurrentAmps = leader.getOutputCurrent();
        inputs.leaderTemperatureCelsius = leader.getMotorTemperature();

        inputs.followerMotorConnected = true;
        inputs.followerPositionMeters = follower.getPosition();
        inputs.followerTargetPositionMeters = follower.getTargetPosition();
        inputs.followerVelocityMetersPerSecond = follower.getVelocity();
        inputs.followerAppliedOutputVolts = follower.getBusVoltage();
        inputs.followerSupplyCurrentAmps = follower.getOutputCurrent();
        inputs.followerTemperatureCelsius = follower.getMotorTemperature();
    }

    @Override
    public void setPosition(double position) {
        leader.setTargetPosition(position);
        follower.setTargetPosition(position);
    }

    @Override
    public void runCharacterization(double input) {
        leader.setVoltage(input);
        follower.setVoltage(input);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        if (brake) {
            leader.setBrakeMode();
            follower.setBrakeMode();
        } else {
            leader.setCoastMode();
            follower.setCoastMode();
        }
    }

    @Override
    public void resetEncoder(double position) {
        leader.resetEncoder(position);
        follower.resetEncoder(position);
    }

    @Override
    public void setGains(GainConstants constants) {
        leader.setPID(ElevatorConstants.elevatorLogged.get());
        follower.setPID(ElevatorConstants.elevatorLogged.get());
    }

    
}
