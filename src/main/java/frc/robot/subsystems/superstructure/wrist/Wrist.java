// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.wrist;

import java.util.function.DoubleSupplier;
import frc.robot.util.Constants.WristConstants;
import frc.robot.util.custom.LoggedTunableBoolean;
import frc.robot.util.custom.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private final LoggedTunableBoolean brakeMotor = new LoggedTunableBoolean("Wrist/BrakeMotor", WristConstants.BRAKE_MOTOR);
    private final LoggedTunableNumber stowPosition = new LoggedTunableNumber("Wrist/StowPostion", WristConstants.STOW_POSITION_RADIANS);
    private final LoggedTunableNumber intakePosition = new LoggedTunableNumber("Wrist/IntakePosition", WristConstants.INTAKE_POSITION_RADIANS);
    private final LoggedTunableNumber l1Position = new LoggedTunableNumber("Wrist/L1Postition", WristConstants.L1_POSITION_RADIANS);
    private final LoggedTunableNumber l2Position = new LoggedTunableNumber("Wrist/L2Postition", WristConstants.L2_POSITION_RADIANS);
    private final LoggedTunableNumber l3Position = new LoggedTunableNumber("Wrist/L3Postition", WristConstants.L3_POSITION_RADIANS);
    private final LoggedTunableNumber l4Position = new LoggedTunableNumber("Wrist/L4Postition", WristConstants.L4_POSITION_RADIANS);

    public Wrist(WristIO io) {
        this.io = io;
        brakeMotor.onChanged(runOnce(() -> this.io.setBrakeMode(brakeMotor.get())));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Wrist", inputs);
        Logger.recordOutput("Subsystems/Wrist/AtDesiredPosition", atDesiredPosition());
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
        return MathUtil.isNear(inputs.targetPositionRads, inputs.encoderPositionRads, WristConstants.WRIST_DEADBAND_RADIANS);
    }
}
