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
import frc.robot.util.hardware.phoenix.Kraken.TelemetryPreference;
import frc.robot.util.Constants;

public class Claw extends SubsystemBase {

    private Kraken motor;
    private boolean hasPiece;

  /** Creates a new Claw. */
    public Claw() {
        motor = new Kraken(ElevatorConstants.CLAW_CAN_ID, "SuperStructure");
        configMotors();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        hasPiece = hasPiece();
    }

    public void configMotors() {
        motor.setBrakeMode(true);
        motor.setTelemetryPreference(TelemetryPreference.NO_ENCODER);
    }

    public Command outtakeCommand() {
        return run(() -> motor.setPercentOutput(ElevatorConstants.CLAW_OUTTAKE));
    }

    public Command intakeCommand() {
        return run(() -> motor.setPercentOutput(ElevatorConstants.CLAW_INTAKE));
    }

    public Command stopCommand() {
        return run(() -> motor.setPercentOutput(0));
    }

    public Command setPercentCommand(double percent) {
        return run(() -> setPercent(percent));
    }

    public void setPercent(double percent) {
        motor.setPercentOutput(percent);
    }

    private boolean hasPiece() {
        return true;
    }

    public boolean getHasPiece() {
        return hasPiece;
    }


}
