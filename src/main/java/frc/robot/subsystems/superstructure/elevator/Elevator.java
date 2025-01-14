// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import frc.robot.RobotContainer;
import frc.robot.util.Constants.ElevatorConstants;
import java.util.function.DoubleSupplier;
import frc.robot.util.custom.LoggedTunableBoolean;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    
    private final LoggedTunableBoolean brakeMotor = new LoggedTunableBoolean("Elevator/BrakeMotor", ElevatorConstants.BRAKE_MOTOR);
    
    public Elevator(ElevatorIO io) {
        this.io = io;
        brakeMotor.onChanged(runOnce(() -> this.io.setBrakeMode(brakeMotor.get())));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Elevator", inputs);
        Logger.recordOutput("Subsystems/Elevator/AtTargetPosition", atTargetPosition());

        RobotContainer.elevatorMech.setLength(ElevatorConstants.ELEVATOR_BASE_HEIGHT_METERS + inputs.leaderPositionMeters);
    }

    public void setPosition(double position) {
        io.setPosition(position);
    }

    public Command setPositionCommand(double position) {
        return run(() -> setPosition(position)).andThen(Commands.waitUntil(this::atTargetPosition));
    }   

    public Command setPositionCommand(DoubleSupplier positionSupplier) {
        return run(() -> setPosition(positionSupplier.getAsDouble())).andThen(Commands.waitUntil(this::atTargetPosition));
    }

    public boolean atTargetPosition() {
        return MathUtil.isNear(inputs.leaderTargetPositionMeters, inputs.leaderPositionMeters, ElevatorConstants.ELEVATOR_DEADBAND_METERS);
    }

}
