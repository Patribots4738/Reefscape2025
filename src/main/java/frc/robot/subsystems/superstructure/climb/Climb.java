// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.climb;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.custom.LoggedTunableBoolean;
import frc.robot.util.custom.LoggedTunableNumber;

public class Climb extends SubsystemBase {
    
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private final LoggedTunableBoolean brakeMotor = new LoggedTunableBoolean("Climb/BrakeMotor", ClimbConstants.BRAKE_MOTOR);
    private final LoggedTunableNumber stowPosition = new LoggedTunableNumber("Climb/StowPosition", ClimbConstants.STOW_POSITION_RADIANS);
    private final LoggedTunableNumber readyPosition = new LoggedTunableNumber("Climb/ReadyPosition", ClimbConstants.READY_POSITION_RADIANS);
    private final LoggedTunableNumber finalPosition = new LoggedTunableNumber("Climb/FinalPosition", ClimbConstants.FINAL_POSITION_RADIANS);
    

    public Climb(ClimbIO io) {
        this.io = io;
        brakeMotor.onChanged(runOnce(() -> this.io.setBrakeMode(brakeMotor.get())));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Climb", inputs);
        Logger.recordOutput("Subsystems/Climb/AtDesiredPosition", atDesiredPosition());
    }

    public void setPosition(double position) {
        io.setPosition(position);
    }

    public Command setPositionCommand(double position) {
        return run(() -> setPosition(position)).andThen(Commands.waitUntil(this::atDesiredPosition));
    }

    public Command setPositionCommand(DoubleSupplier positionSupplier) {
        return run(() -> setPosition(positionSupplier.getAsDouble())).andThen(Commands.waitUntil(this::atDesiredPosition));
    }

    public Command stowPositionCommand() {
        return setPositionCommand(stowPosition::get);
    }

    public Command readyPositionCommand() {
        return setPositionCommand(readyPosition::get);
    }

    public Command finalPositionCommand() {
        return setPositionCommand(finalPosition::get);
    }

    public boolean atDesiredPosition() {
        return MathUtil.isNear(inputs.leaderTargetPositionRads, inputs.encoderPositionRads, ClimbConstants.CLIMB_DEADBAND_RADIANS);
    }

}
