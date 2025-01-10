// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.TelemetryPreference;

public class Claw extends SubsystemBase {

    private final Kraken motor;
    private boolean hasPiece = false;

    public Claw() {
        motor = new Kraken(ElevatorConstants.CLAW_CAN_ID, "SuperStructure");
        configMotor();
    }

    @Override
    public void periodic() {
        motor.refreshSignals();
        hasPiece = hasPiece();
    }

    public void configMotor() {
        motor.setBrakeMode(true);
        motor.setTelemetryPreference(TelemetryPreference.NO_ENCODER);
    }

    public void setPercent(double percent) {
        motor.setPercentOutput(percent);
    }

    public Command setPercentCommand(double percent) {
        return run(() -> setPercent(percent));
    }

    public Command outtakeCommand() {
        return setPercentCommand(ElevatorConstants.CLAW_OUTTAKE_PERCENT);
    }

    public Command intakeCommand() {
        return setPercentCommand(ElevatorConstants.CLAW_INTAKE_PERCENT);
    }

    public Command stopCommand() {
        return setPercentCommand(0.0);
    }

    private boolean hasPiece() {
        return true;
    }

    public boolean getHasPiece() {
        return hasPiece;
    }

}
