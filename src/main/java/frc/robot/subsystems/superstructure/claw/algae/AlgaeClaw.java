// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.claw.algae;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.claw.ClawIO;
import frc.robot.subsystems.superstructure.claw.ClawIOInputsAutoLogged;
import frc.robot.util.Constants.AlgaeClawConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.Constants.LoggingConstants.Mode;

public class AlgaeClaw extends SubsystemBase {

    private final ClawIO io;
    private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

    private double percentOutput = 0.0;
    private boolean shouldRunSetpoint = false;
    private boolean hasPiece = false;

    private final Debouncer hasPieceDebouncerRising;
    private final Debouncer hasPieceDebouncerFalling;
    
    public AlgaeClaw(ClawIO io) {
        this.io = io;
        hasPieceDebouncerRising = new Debouncer(0.05, DebounceType.kRising);
        hasPieceDebouncerFalling = new Debouncer(0.25, DebounceType.kFalling);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/AlgaeClaw", inputs);
        Logger.recordOutput("Subsystems/AlgaeClaw/HasPiece", hasPiece());

        boolean hasPieceUnfiltered = MathUtil.isNear(AlgaeClawConstants.CURRENT_LIMIT, inputs.torqueCurrentAmps, 25d) && inputs.velocityRotationsPerSec < 60d;
        Logger.recordOutput("Subsystems/AlgaeClaw/HasPieceUnfiltered", hasPieceUnfiltered);
        if (hasPiece) {
            hasPiece = hasPieceDebouncerFalling.calculate(hasPieceUnfiltered);
        } else {
            hasPiece = hasPieceDebouncerRising.calculate(hasPieceUnfiltered);
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
    
    public boolean hasPiece() {
        return (LoggingConstants.getMode() != Mode.REPLAY && !FieldConstants.IS_REAL) ? percentOutput > 0 : hasPiece;
    }
}

