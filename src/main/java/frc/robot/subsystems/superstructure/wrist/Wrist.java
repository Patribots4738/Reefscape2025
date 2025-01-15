// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.wrist;

import java.util.function.DoubleSupplier;

import frc.robot.RobotContainer;
import frc.robot.util.Constants.WristConstants;
import frc.robot.util.custom.LoggedTunableBoolean;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
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
        brakeMotor.onChanged(runOnce(() -> this.io.setBrakeMode(brakeMotor.get())));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Wrist", inputs);
        Logger.recordOutput("Subsystems/Wrist/AtTargetPosition", atTargetPosition());

        RobotContainer.wristMech.setAngle(180 + Units.radiansToDegrees(inputs.encoderPositionRads));
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

    public boolean atTargetPosition() {
        return MathUtil.isNear(targetPosition, inputs.encoderPositionRads, WristConstants.WRIST_DEADBAND_RADIANS);
    }
}
