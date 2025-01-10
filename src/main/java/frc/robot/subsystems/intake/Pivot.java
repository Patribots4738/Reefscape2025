// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.hardware.phoenix.CANCoderCustom;
import frc.robot.util.hardware.phoenix.Kraken;

public class Pivot extends SubsystemBase {

    private Kraken motor;
    private CANCoderCustom turnEncoder;
    private boolean atDesired = false;

    /** Creates a new Pivot. */
    public Pivot() {
        motor = new Kraken(IntakeConstants.PIVOT_CAN_ID, "SuperStructure");
        turnEncoder = new CANCoderCustom(IntakeConstants.PIVOT_CANCODER_ID, "SuperStructure");
        configMotors();
    }

    public void configMotors() {
        motor.setEncoder(turnEncoder.getDeviceID(), IntakeConstants.PIVOT_GEAR_RATIO);
        motor.setBrakeMode(true);
    }

    public void configEncoder() {
        turnEncoder.configureMagnetSensor(false, IntakeConstants.PIVOT_ENCODER_OFFSET);
        turnEncoder.setPositionConversionFactor(IntakeConstants.PIVOT_ENCODER_POSITION_FACTOR);
        turnEncoder.setVelocityConversionFactor(IntakeConstants.PIVOT_ENCODER_VELOCITY_FACTOR);
    }

    @Override
    public void periodic() {
        atDesired = atDesired();
    }

    public void setPosition(double position) {
        motor.setTargetPosition(position);
    }

    public Command setPositionCommand(double position) {
        return run(() -> setPosition(position));
    }

    public Command stowPositionCommand() {
        return setPositionCommand(IntakeConstants.PIVOT_STOW_RADIANS);
    }

    public Command handoffPositionCommand() {
        return setPositionCommand(IntakeConstants.PIVOT_HANDOFF_RADIANS);
    }

    public Command intakePositionCommand() {
        return setPositionCommand(IntakeConstants.PIVOT_INTAKE_RADIANS);
    }

    public boolean atDesired() {
        return true;
    }

    public boolean getAtDesired() {
        return atDesired;
    }

}
