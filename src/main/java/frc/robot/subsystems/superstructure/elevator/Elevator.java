// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import frc.robot.util.Constants.ElevatorConstants;
import java.util.function.DoubleSupplier;
import frc.robot.util.custom.LoggedTunableBoolean;
import frc.robot.util.custom.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final LoggedTunableBoolean brakeMotor = new LoggedTunableBoolean("Elevator/BrakeMotor", ElevatorConstants.BRAKE_MOTOR);
    private final LoggedTunableNumber stowPosition = new LoggedTunableNumber("Elevator/StowPostion", ElevatorConstants.STOW_POSITION_METERS);
    private final LoggedTunableNumber intakePosition = new LoggedTunableNumber("Elevator/IntakePosition", ElevatorConstants.INTAKE_POSITION_METERS);
    private final LoggedTunableNumber l1Position = new LoggedTunableNumber("Elevator/L1Postition", ElevatorConstants.L1_POSITION_METERS);
    private final LoggedTunableNumber l2Position = new LoggedTunableNumber("Elevator/L2Postition", ElevatorConstants.L2_POSITION_METERS);
    private final LoggedTunableNumber l3Position = new LoggedTunableNumber("Elevator/L3Postition", ElevatorConstants.L3_POSITION_METERS);
    private final LoggedTunableNumber l4Position = new LoggedTunableNumber("Elevator/L4Postition", ElevatorConstants.L4_POSITION_METERS);
    
    public Elevator(ElevatorIO io) {
        this.io = io;
        brakeMotor.onChanged(runOnce(() -> this.io.setBrakeMode(brakeMotor.get())));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Elevator", inputs);
        Logger.recordOutput("Subsystems/Elevator/AtDesiredPosition", atDesiredPosition());
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

    public Command intakePositionCommand() {
        return setPositionCommand(intakePosition::get);
    }

    public Command l1PositionCommand() {
        return setPositionCommand(l1Position::get);
    }

    public Command l2PositionCommand() {
        return setPositionCommand(l2Position::get);
    }

    public Command l3PositionCommand() {
        return setPositionCommand(l3Position::get);
    }

    public Command l4PositionCommand() {
        return setPositionCommand(l4Position::get);
    }

    public boolean atDesiredPosition() {
        return MathUtil.isNear(inputs.leaderTargetPositionMeters, inputs.leaderPositionMeters, ElevatorConstants.ELEVATOR_DEADBAND_METERS);
    }

}
