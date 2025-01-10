// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.hardware.phoenix.CANCoderCustom;
import frc.robot.util.hardware.phoenix.Kraken;

public class Wrist extends SubsystemBase {
    private final Kraken motor;
    private final CANCoderCustom encoder;
    private boolean atDesired = false;

    public Wrist() {
        motor = new Kraken(ElevatorConstants.WRIST_CAN_ID, "SuperStructure");
        encoder = new CANCoderCustom(ElevatorConstants.WRIST_CANCODER_ID, "SuperStructure");
        configEncoder();
        configMotor();
    }

    @Override
    public void periodic() {
        encoder.refreshSignals();
        motor.refreshSignals();
        atDesired = atDesired();
    }

    public void configMotor() {
        motor.setGains(ElevatorConstants.WRIST_GAINS);
        motor.setEncoder(ElevatorConstants.WRIST_CANCODER_ID, ElevatorConstants.WRIST_GEAR_RATIO);
        motor.setBrakeMode(true);
    }

    public void configEncoder() {
        encoder.configureMagnetSensor(false, ElevatorConstants.WRIST_ENCODER_OFFSET);
        encoder.setPositionConversionFactor(ElevatorConstants.WRIST_POSITION_FACTOR);
        encoder.setVelocityConversionFactor(ElevatorConstants.WRIST_VELOCITY_FACTOR);
    }

    public void setPosition(double position) {
        motor.setTargetPosition(position);
    }

    public Command setPositionCommand(double position) {
        return runOnce(() -> setPosition(position)).andThen(Commands.waitUntil(this::getAtDesired));
    }

    public Command setPositionCommand(DoubleSupplier positionSupplier) {
        return runOnce(() -> setPosition(positionSupplier.getAsDouble())).andThen(Commands.waitUntil(this::getAtDesired));
    }

    public Command handoffPositionCommand() {
        return setPositionCommand(ElevatorConstants.WRIST_HANDOFF_RADIANS);
    }

    public Command intakePositionCommand() {
        return setPositionCommand(ElevatorConstants.WRIST_INTAKE_RADIANS);
    }

    public Command stowPositionCommand() {
        return setPositionCommand(ElevatorConstants.WRIST_STOW_RADIANS);
    }

    public boolean atDesired() {
        return motor.getPositionAsDouble() == motor.getTargetPosition();
    }

    public boolean getAtDesired() {
        return atDesired;
    }

}
