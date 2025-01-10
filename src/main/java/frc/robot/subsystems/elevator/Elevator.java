// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.hardware.phoenix.CANCoderCustom;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.Constants;

public class Elevator extends SubsystemBase {

    private Kraken motor;
    private CANCoderCustom encoder;
    private boolean atDesired;

  /** Creates a new Elevator. */
    public Elevator() {
        motor = new Kraken(ElevatorConstants.ELEVATOR_CAN_ID, "SuperStructure");
        encoder = new CANCoderCustom(ElevatorConstants.ELEVATOR_CANCODER_ID, "SuperStructure");
        configMotors(); 

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        atDesired = atDesired();
    }

    public void configMotors() {
        motor.setGains(ElevatorConstants.ELEVATOR_PID);
        motor.setEncoder(ElevatorConstants.ELEVATOR_CANCODER_ID, ElevatorConstants.ELEVATOR_GEAR_RATIO);
        motor.setBrakeMode(true);
    }

    public void configEncoder() {
        encoder.configureMagnetSensor(false, ElevatorConstants.ELEVATOR_ENCODER_OFFSET);
        encoder.setPositionConversionFactor(ElevatorConstants.ELEVATOR_POSITION_FACTOR);
        encoder.setVelocityConversionFactor(ElevatorConstants.ELEVATOR_VELOCITY_FACTOR);
    }


    public boolean atDesired() {
        return motor.getPositionAsDouble() == motor.getTargetPosition(); 
    }

    public boolean getAtDesired() {
        return atDesired;
    }

    public Command setPositionCommand(double position) {
        return run(() -> motor.setTargetPosition(position));
    }
}
