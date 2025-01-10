// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.hardware.phoenix.CANCoderCustom;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.Constants;

public class Wrist extends SubsystemBase {
    private Kraken motor;
    private CANCoderCustom encoder;
    private boolean atDesired;
    
  /** Creates a new Wrist. */
    public Wrist() {
        motor = new Kraken(ElevatorConstants.WRIST_CAN_ID, "superStructure");
        encoder = new CANCoderCustom(ElevatorConstants.WRIST_CANCODER_ID, "SuperStructure"); 
        configMotors();

    }

        @Override
        public void periodic() {
            atDesired = atDesired();
    // This method will be called once per scheduler run
        }

        public void configMotors() {
            motor.setGains(ElevatorConstants.WRIST_PID);
            motor.setEncoder(ElevatorConstants.WRIST_CANCODER_ID, ElevatorConstants.WRIST_GEAR_RATIO);
            motor.setBrakeMode(true);
        }

        public void configEncoder() {
            encoder.configureMagnetSensor(false, ElevatorConstants.WRIST_ENCODER_OFFSET);
            encoder.setPositionConversionFactor(ElevatorConstants.WRIST_POSITION_FACTOR);
            encoder.setVelocityConversionFactor(ElevatorConstants.WRIST_VELOCITY_FACTOR);
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
