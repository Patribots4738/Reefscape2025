// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.IntakeConstants;

import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.TelemetryPreference;

public class Intake extends SubsystemBase {

    private final Kraken motor;
    private boolean hasPiece = false;

    public Intake() {
        motor = new Kraken(IntakeConstants.INTAKE_CAN_ID, "SuperStructure");
        configMotor();
    }

    private void configMotor() {
        motor.setBrakeMode(true);
        motor.setTelemetryPreference(TelemetryPreference.NO_ENCODER);
    }

    @Override
    public void periodic() {
        motor.refreshSignals();
        hasPiece = hasPiece();
    }

    public void setPercent(double percent) {
        motor.setPercentOutput(percent);
    }

    public Command setPercentCommand(double percent) {
        return run(() -> setPercent(percent));
    }

    public Command intakeCommand() {
        return setPercentCommand(IntakeConstants.INTAKE_PERCENT);
    }

    public Command outtakeCommand() {
        return setPercentCommand(IntakeConstants.OUTTAKE_PERCENT);
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
