// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.claw.coral;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.claw.ClawIO;
import frc.robot.subsystems.superstructure.claw.ClawIOInputsAutoLogged;
import frc.robot.util.Constants.CoralClawConstants;
import frc.robot.util.custom.LoggedTunableBoolean;
import frc.robot.util.custom.LoggedTunableNumber;

public class CoralClaw extends SubsystemBase {

    private final ClawIO io;
    private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();
    
    private final LoggedTunableBoolean brakeMotor = new LoggedTunableBoolean("CoralClaw/BrakeMotor", CoralClawConstants.BRAKE_MOTOR);
    private final LoggedTunableNumber intakePercent = new LoggedTunableNumber("CoralClaw/IntakePercent", CoralClawConstants.INTAKE_PERCENT);
    private final LoggedTunableNumber outtakePercent = new LoggedTunableNumber("CoralClaw/OuttakePercent", CoralClawConstants.OUTTAKE_PERCENT);

    private double percentOutput = 0.0;
    private boolean shouldRunSetpoint = false;
    
    public CoralClaw(ClawIO io) {
        this.io = io;
        brakeMotor.onChanged(runOnce(() -> this.io.setBrakeMode(brakeMotor.get())).ignoringDisable(true));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/AlgaeClaw", inputs);
        Logger.recordOutput("Subsystems/AlgaeClaw/HasPiece", hasPiece());

        // Run setpoint on RIO to minimize CAN utilization
        if (shouldRunSetpoint) {
            io.setPercent(percentOutput);
        } else {
            io.setNeutral();
        }
    }

    public void setPercent(double percent) {
        percentOutput = percent;
        shouldRunSetpoint = true;
    }

    public void setNeutral() {
        shouldRunSetpoint = false;
    }

    public Command setPercentCommand(DoubleSupplier percentSupplier) {
        return runOnce(() -> setPercent(percentSupplier.getAsDouble()));
    }

    public Command setPercentCommand(double percent) {
        return setPercentCommand(() -> percent);
    }

    public Command setNeutralCommand() {
        return runOnce(this::setNeutral);
    }

    public Command intakeCommand() {
        return setPercentCommand(intakePercent::get);
    }

    public Command outtakeCommand() {
        return setPercentCommand(outtakePercent::get);
    }

    public Command stopCommand() {
        return setPercentCommand(0.0);
    }

    public Command outtakeTimeCommand(double time){
        return Commands.sequence(
            outtakeCommand(),
            Commands.waitSeconds(time),
            stopCommand()
        );
    }

    public boolean hasPiece() {
        // TODO: IMPLEMENT DETECTION LOGIC
        return false;
    }
}
