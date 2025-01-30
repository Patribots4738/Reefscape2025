// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import frc.robot.RobotContainer;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.Constants.LoggingConstants;

import java.util.function.DoubleSupplier;
import frc.robot.util.custom.LoggedTunableBoolean;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    
    private final LoggedTunableBoolean brakeMotor = new LoggedTunableBoolean("Elevator/BrakeMotor", ElevatorConstants.BRAKE_MOTOR);

    private double targetPosition = 0.0;
    
    public Elevator(ElevatorIO io) {
        this.io = io;
        brakeMotor.onChanged(runOnce(() -> this.io.setBrakeMode(brakeMotor.get())));
        ElevatorConstants.LOGGED_ELEVATOR_GAINS.onChanged(Commands.run(() -> io.setGains(ElevatorConstants.LOGGED_ELEVATOR_GAINS.get())));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Elevator", inputs);
        Logger.recordOutput("Subsystems/Elevator/AtTargetPosition", atTargetPosition());

        RobotContainer.components3d[LoggingConstants.ELEVATOR_FIRST_STAGE_INDEX] = new Pose3d(
            0, 
            0, 
            inputs.leaderPositionMeters, 
            new Rotation3d()
        );
        RobotContainer.components3d[LoggingConstants.ELEVATOR_SECOND_STAGE_INDEX] = new Pose3d(
            0, 
            0, 
            inputs.leaderPositionMeters * 2.0, 
            new Rotation3d()
        );
        RobotContainer.components3d[LoggingConstants.WRIST_INDEX] = new Pose3d(
            LoggingConstants.WRIST_OFFSET.getX(),
            LoggingConstants.WRIST_OFFSET.getY(),
            LoggingConstants.WRIST_OFFSET.getZ() + inputs.leaderPositionMeters * 2.0,
            RobotContainer.components3d[LoggingConstants.WRIST_INDEX].getRotation()
        );
    }

    public void setPosition(double position) {
        position = MathUtil.clamp(position, 0, ElevatorConstants.MAX_DISPLACEMENT_METERS);
        targetPosition = position;
        io.setPosition(position);

        RobotContainer.desiredComponents3d[LoggingConstants.ELEVATOR_FIRST_STAGE_INDEX] = new Pose3d(
            0, 
            0, 
            position, 
            new Rotation3d()
        );
        RobotContainer.desiredComponents3d[LoggingConstants.ELEVATOR_SECOND_STAGE_INDEX] = new Pose3d(
            0, 
            0, 
            position * 2.0, 
            new Rotation3d()
        );
        RobotContainer.desiredComponents3d[LoggingConstants.WRIST_INDEX] = new Pose3d(
            LoggingConstants.WRIST_OFFSET.getX(),
            LoggingConstants.WRIST_OFFSET.getY(),
            LoggingConstants.WRIST_OFFSET.getZ() + position * 2.0,
            RobotContainer.desiredComponents3d[LoggingConstants.WRIST_INDEX].getRotation()
        );
    }

    public Command setPositionCommand(DoubleSupplier positionSupplier) {
        return runOnce(() -> setPosition(positionSupplier.getAsDouble())).andThen(Commands.waitUntil(this::atTargetPosition));
    }

    public Command setPositionCommand(double position) {
        return setPositionCommand(() -> position);
    }   

    public boolean atPosition(double position) {
        return MathUtil.isNear(position, inputs.leaderPositionMeters, ElevatorConstants.ELEVATOR_DEADBAND_METERS);
    }

    public boolean atTargetPosition() {
        return atPosition(targetPosition);
    }

    public double getPosition() {
        return inputs.leaderPositionMeters;
    }

    public double getCharacterizationVelocity() {
        return inputs.leaderVelocityMetersPerSecond / ElevatorConstants.VELOCITY_CONVERSION_FACTOR;
    }

    public void runCharacterization(double input) {
        io.runCharacterization(input);
    }
}
