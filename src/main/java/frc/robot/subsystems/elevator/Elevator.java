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

public class Elevator extends SubsystemBase {

    private final Kraken motor;
    private final CANCoderCustom encoder;
    private boolean atDesired = false;

    public Elevator() {
        motor = new Kraken(ElevatorConstants.ELEVATOR_CAN_ID, "SuperStructure");
        encoder = new CANCoderCustom(ElevatorConstants.ELEVATOR_CANCODER_ID, "SuperStructure");
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
        motor.setGains(ElevatorConstants.ELEVATOR_GAINS);
        motor.setEncoder(ElevatorConstants.ELEVATOR_CANCODER_ID, ElevatorConstants.ELEVATOR_GEAR_RATIO);
        motor.setBrakeMode(true);
    }

    public void configEncoder() {
        encoder.configureMagnetSensor(false, ElevatorConstants.ELEVATOR_ENCODER_OFFSET);
        encoder.setPositionConversionFactor(ElevatorConstants.ELEVATOR_POSITION_FACTOR);
        encoder.setVelocityConversionFactor(ElevatorConstants.ELEVATOR_VELOCITY_FACTOR);
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
