// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.climb;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.WristConstants;
import frc.robot.util.custom.LoggedTunableBoolean;
import frc.robot.util.custom.LoggedTunableNumber;

public class Climb extends SubsystemBase {
    
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private final LoggedTunableBoolean brakeMotor = new LoggedTunableBoolean("Climb/BrakeMotor", ClimbConstants.BRAKE_MOTOR);
    private final LoggedTunableNumber stowPosition = new LoggedTunableNumber("Climb/StowPosition", ClimbConstants.STOW_POSITION_RADIANS);
    private final LoggedTunableNumber readyPosition = new LoggedTunableNumber("Climb/ReadyPosition", ClimbConstants.READY_POSITION_RADIANS);
    private final LoggedTunableNumber latchPosition = new LoggedTunableNumber("Climb/LatchPosition", ClimbConstants.LATCH_POSITION_RADIANS);
    private final LoggedTunableNumber finalPosition = new LoggedTunableNumber("Climb/FinalPosition", ClimbConstants.FINAL_POSITION_RADIANS);

    private double targetPosition = 0.0;

    public Climb(ClimbIO io) {
        this.io = io;
        brakeMotor.onChanged(runOnce(() -> this.io.setBrakeMode(brakeMotor.get())));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Climb", inputs);
        Logger.recordOutput("Subsystems/Climb/AtDesiredPosition", atTargetPosition());
    }

    public void setPosition(double position) {
        targetPosition = position;
        io.setPosition(position);
    }

    public Command setPositionCommand(DoubleSupplier positionSupplier) {
        return runOnce(() -> setPosition(positionSupplier.getAsDouble())).andThen(Commands.waitUntil(this::atTargetPosition));
    }

    public Command setPositionCommand(double position) {
        return setPositionCommand(() -> position);
    }

    public Command stowPositionCommand() {
        return setPositionCommand(stowPosition::get);
    }

    public Command readyPositionCommand() {
        return setPositionCommand(readyPosition::get);
    }

    public Command latchPositionCommand() {
        return setPositionCommand(latchPosition::get);
    }

    public Command finalPositionCommand() {
        return setPositionCommand(finalPosition::get);
    }

    public boolean atTargetPosition() {
        return MathUtil.isNear(targetPosition, inputs.encoderPositionRads, ClimbConstants.CLIMB_DEADBAND_RADIANS);
    }

    public double getCharacterizationVelocity() {
        return inputs.leaderVelocityRadsPerSec / ClimbConstants.VELOCITY_CONVERSION_FACTOR;
    }

    public void runCharacterization(double input) {
        io.runCharacterization(input);
    }
}
