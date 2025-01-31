// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.wrist;

import java.util.function.DoubleSupplier;

import frc.robot.RobotContainer;
import frc.robot.util.Constants.WristConstants;
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.custom.LoggedTunableBoolean;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private final LoggedTunableBoolean brakeMotor = new LoggedTunableBoolean("Wrist/BrakeMotor", WristConstants.BRAKE_MOTOR);
    

    private double targetPosition = 0.0;

    public Wrist(WristIO io) {
        this.io = io;
        brakeMotor.onChanged(runOnce(() -> this.io.setBrakeMode(brakeMotor.get())).ignoringDisable(true));
        WristConstants.LOGGED_GAINS.onChanged(runOnce(() -> io.setGains(WristConstants.LOGGED_GAINS.get())).ignoringDisable(true));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Wrist", inputs);
        Logger.recordOutput("Subsystems/Wrist/AtTargetPosition", atTargetPosition());

        RobotContainer.components3d[LoggingConstants.WRIST_INDEX] = new Pose3d(
            RobotContainer.components3d[LoggingConstants.WRIST_INDEX].getX(), 
            RobotContainer.components3d[LoggingConstants.WRIST_INDEX].getY(),
            RobotContainer.components3d[LoggingConstants.WRIST_INDEX].getZ(),
            new Rotation3d(0, inputs.positionRads, 0)
        );

    }

    public void setPosition(double position) {
        position = MathUtil.clamp(position, WristConstants.MIN_ANGLE_RADIANS, WristConstants.MAX_ANGLE_RADIANS);
        targetPosition = position;
        io.setPosition(position);

        RobotContainer.desiredComponents3d[LoggingConstants.WRIST_INDEX] = new Pose3d(
            RobotContainer.desiredComponents3d[LoggingConstants.WRIST_INDEX].getX(), 
            RobotContainer.desiredComponents3d[LoggingConstants.WRIST_INDEX].getY(),
            RobotContainer.desiredComponents3d[LoggingConstants.WRIST_INDEX].getZ(),
            new Rotation3d(0, position, 0)
        );
    }

    public Command setPositionCommand(DoubleSupplier positionSupplier) {
        return runOnce(() -> setPosition(positionSupplier.getAsDouble())).andThen(Commands.waitUntil(this::atTargetPosition));
    }

    public Command setPositionCommand(double position) {
        return setPositionCommand(() -> position);
    }   

    public boolean atPosition(double position) {
        return MathUtil.isNear(position, inputs.positionRads, WristConstants.DEADBAND_RADIANS);
    }

    public boolean atTargetPosition() {
        return atPosition(targetPosition);
    }

    public double getPosition() {
        return inputs.positionRads;
    }

    public double getCharacterizationVelocity() {
        return inputs.velocityRadsPerSec / WristConstants.VELOCITY_CONVERSION_FACTOR;
    }

    public void runCharacterization(double input) {
        io.runCharacterization(input);
    }


}
