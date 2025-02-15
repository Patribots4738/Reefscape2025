// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.claw.coral;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;
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
    private boolean hasPiece;;

    private final Debouncer hasPieceDebouncer;
    
    public CoralClaw(ClawIO io) {
        this.io = io;
        brakeMotor.onChanged(runOnce(() -> this.io.setBrakeMode(brakeMotor.get())).ignoringDisable(true));
        hasPieceDebouncer = new Debouncer(0.25);
        hasPiece = DriverStation.isFMSAttached();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/CoralClaw", inputs);

        // If claw is running, update hasPiece with timed debouncing function.
        // If it isn't running, assume the coral is in the same state that it was when the claw stopped.
        if (Robot.gameMode == GameMode.DISABLED && inputs.velocityRotationsPerSecond > 0 && inputs.statorCurrentAmps < 10) {
            hasPiece = true;
        } else if (percentOutput != 0.0) {
            hasPiece = hasPieceDebouncer.calculate(MathUtil.isNear(CoralClawConstants.CURRENT_LIMIT, inputs.statorCurrentAmps, 10));
        }

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

    @AutoLogOutput (key = "Subsystems/CoralClaw/HasCoral")
    public boolean hasPiece() {
        return hasPiece;
    }
}
