// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.claw;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.ClawConstants;
import frc.robot.util.custom.LoggedTunableBoolean;
import frc.robot.util.custom.LoggedTunableNumber;

public class Claw extends SubsystemBase {

    private final ClawIO io;
    private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();
    
    private final LoggedTunableBoolean brakeMotor = new LoggedTunableBoolean("Claw/BrakeMotor", ClawConstants.BRAKE_MOTOR);
    private final LoggedTunableNumber intakePercent = new LoggedTunableNumber("Claw/IntakePercent", ClawConstants.INTAKE_PERCENT);
    private final LoggedTunableNumber outtakePercent = new LoggedTunableNumber("Claw/OuttakePercent", ClawConstants.OUTTAKE_PERCENT);
    
    public Claw(ClawIO io) {
        this.io = io;
        brakeMotor.onChanged(runOnce(() -> this.io.setBrakeMode(brakeMotor.get())));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Claw", inputs);
        Logger.recordOutput("Subsystems/Claw/HasPiece", hasPiece());
    }

    public void setPercent(double percent) {
        io.setPercent(percent);
    }

    public Command setPercentCommand(DoubleSupplier percentSupplier) {
        return run(() -> setPercent(percentSupplier.getAsDouble()));
    }

    public Command setPercentCommand(double percent) {
        return setPercentCommand(() -> percent);
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

    public boolean hasPiece() {
        // TODO: IMPLEMENT DETECTION LOGIC
        return false;
    }
}
