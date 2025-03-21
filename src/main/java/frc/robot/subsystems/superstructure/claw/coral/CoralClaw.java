// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.claw.coral;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;
import frc.robot.subsystems.superstructure.claw.ClawIO;
import frc.robot.subsystems.superstructure.claw.ClawIOInputsAutoLogged;
import frc.robot.util.Constants.CoralClawConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.Constants.LoggingConstants.Mode;

public class CoralClaw extends SubsystemBase {

    private final ClawIO io;
    private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

    private double percentOutput = 0.0;
    private boolean shouldRunSetpoint = false;
    private boolean hasPiece;

    private final Debouncer hasPieceDebouncer;
    
    public CoralClaw(ClawIO io) {
        this.io = io;
        hasPieceDebouncer = new Debouncer(0.1, DebounceType.kBoth);
        hasPiece = DriverStation.isFMSAttached();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/CoralClaw", inputs);

        updateHasPiece();
        Logger.recordOutput("Subsystems/CoralClaw/HasCoral", hasPiece());

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

    public void updateHasPiece() {
        if (FieldConstants.IS_SIMULATION && inputs.percentOutput != 0.0 && LoggingConstants.getMode() != Mode.REPLAY) {
            hasPiece = inputs.percentOutput > 0.0;
        } else if (Robot.gameMode == GameMode.DISABLED && inputs.velocityRotationsPerSec != 0) {
            hasPiece = hasPieceDebouncer.calculate(inputs.velocityRotationsPerSec > 0) || (DriverStation.isFMSAttached() && !Robot.exitAuto);
        } else if (Robot.gameMode != GameMode.DISABLED && percentOutput != 0.0) {
            if (percentOutput > 0 && !hasPiece) {
                hasPiece = hasPieceDebouncer.calculate(
                    MathUtil.isNear(CoralClawConstants.CURRENT_LIMIT, inputs.torqueCurrentAmps, CoralClawConstants.CORAL_CLAW_CURRENT_DEADBAND) 
                    && Math.abs(inputs.velocityRotationsPerSec) < CoralClawConstants.HAS_PIECE_INTAKE_THRESHOLD);
            } else {
                hasPiece = Math.abs(inputs.velocityRotationsPerSec) < CoralClawConstants.HAS_PIECE_OUTTAKE_THRESHOLD_1;
            }
        }
    }

    @AutoLogOutput (key = "Subsystems/CoralClaw/HasCoral")
    public boolean hasPiece() {
        return hasPiece;    
    }
}
