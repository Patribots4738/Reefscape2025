// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.module;
import frc.robot.util.Constants.MAXSwerveModuleConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.rev.Neo;

public class ModuleIONeo implements ModuleIO {
    private final Neo driveMotor;
    private final Neo turnMotor;

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public ModuleIONeo(int drivingCANId, int turningCANId) {
        driveMotor = new Neo(drivingCANId, true);
        
        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.

        turnMotor = new Neo(turningCANId, false, MAXSwerveModuleConstants.TURNING_ENCODER_INVERTED, true);
        resetDriveEncoder();
        configMotors();
    }

    public void configMotors() {
        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        driveMotor.setPositionConversionFactor(MAXSwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
        driveMotor.setVelocityConversionFactor(MAXSwerveModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        turnMotor.setPositionConversionFactor(MAXSwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
        turnMotor.setVelocityConversionFactor(MAXSwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        turnMotor.enablePIDWrapping(
            MAXSwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT,
            MAXSwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

        setGains(MAXSwerveModuleConstants.DRIVING_PID, MAXSwerveModuleConstants.TURNING_PID);

        driveMotor.setSmartCurrentLimit(MAXSwerveModuleConstants.NEO_CURRENT_LIMIT);
        turnMotor.setSmartCurrentLimit(MAXSwerveModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);

        setDriveBrakeMode(true);
        setTurnBrakeMode(true);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // swerve module position and state

        // drive motor
        inputs.drivePositionMeters = driveMotor.getPosition();
        inputs.driveVelocityMPS = driveMotor.getVelocity();
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput();
        inputs.driveSupplyCurrentAmps = driveMotor.getOutputCurrent();


        // turning motor
        inputs.turnAppliedVolts = turnMotor.getAppliedOutput();
        inputs.turnInternalPositionRads = turnMotor.getPosition();
        inputs.turnInternalVelocityRadsPerSec = turnMotor.getVelocity();
        inputs.turnSupplyCurrentAmps = turnMotor.getOutputCurrent();

        // Fake it till you make it or something
        inputs.turnEncoderAbsPositionRads = turnMotor.getPosition();

    }

    /**
     * Resets drive encoder to 0.
     */
    @Override
    public void resetDriveEncoder()  {
        driveMotor.resetEncoder(0);
    }

    @Override
    public void setDriveBrakeMode(boolean brake) {
        if (brake) {
            driveMotor.setBrakeMode();
        } else {
            driveMotor.setCoastMode();
        }
    }

    @Override
    public void setTurnBrakeMode(boolean brake) {
        if (brake) {
            turnMotor.setBrakeMode();
        } else {
            turnMotor.setCoastMode();
        }
    }

    @Override
    public void runDriveCharacterization(double input, double turnAngle) {
        turnMotor.setTargetPosition(turnAngle);
        driveMotor.setVoltage(input);
    }

    @Override
    public void runTurnCharacterization(double input) {
        turnMotor.setVoltage(input);
    }

    @Override
    public void runDriveVelocity(double velocity, double feedforward) {
        driveMotor.setTargetVelocity(velocity, feedforward);
    }

    @Override
    public void setTurnPosition(double position) {
        turnMotor.setTargetPosition(position);
    }

    @Override
    public void setTurnVelocity(double velocity) {
        turnMotor.setTargetVelocity(velocity);
    }

    @Override
    public void setGains(GainConstants driveGains, GainConstants turnGains) {
        driveMotor.setPID(driveGains);
        turnMotor.setPID(turnGains);
    }
}