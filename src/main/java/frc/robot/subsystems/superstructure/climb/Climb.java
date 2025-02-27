// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.climb;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.LoggingConstants;

public class Climb extends SubsystemBase {
    
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private double targetPosition = 0.0;

    public Climb(ClimbIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Climb", inputs);
        Logger.recordOutput("Subsystems/Climb/AtDesiredPosition", atTargetPosition());

        RobotContainer.components3d[LoggingConstants.CLIMB_INDEX] = new Pose3d(
            LoggingConstants.CLIMB_OFFSET, 
            new Rotation3d(-inputs.positionRads, 0, 0)
        );
    }

    public void setPosition(double position, boolean slam) {
        position = MathUtil.clamp(position, ClimbConstants.MIN_ANGLE_RADIANS, ClimbConstants.MAX_ANGLE_RADIANS);
        targetPosition = position;
        io.setPosition(targetPosition, slam ? 1 : 0);

        RobotContainer.desiredComponents3d[LoggingConstants.CLIMB_INDEX] = new Pose3d(
            LoggingConstants.CLIMB_OFFSET,
            new Rotation3d(-position, 0, 0)
        );
    }

    public void setNeutral() {
        io.setNeutral();
    }

    public void resetEncoder() {
        io.resetEncoder(0);
    }

    public Command setPositionCommand(DoubleSupplier positionSupplier, BooleanSupplier slamSupplier) {
        return runOnce(() -> setPosition(positionSupplier.getAsDouble(), slamSupplier.getAsBoolean())).andThen(Commands.waitUntil(this::atTargetPosition));
    }

    public Command setPositionCommand(double position, boolean slam) {
        return setPositionCommand(() -> position, () -> slam);
    }

    public Command setNeutralCommand() {
        return runOnce(this::setNeutral);
    }

    public Command resetEncoderCommand() {
        return runOnce(this::resetEncoder);
    }

    public boolean atPosition(double position) {
        return MathUtil.isNear(position, inputs.positionRads, ClimbConstants.DEADBAND_RADIANS);
    }

    public boolean atTargetPosition() {
        return atPosition(targetPosition);
    }

    public double getPosition() {
        return inputs.positionRads;
    }

    public double getCharacterizationVelocity() {
        return inputs.velocityRadsPerSec / ClimbConstants.VELOCITY_CONVERSION_FACTOR;
    }

    public void runCharacterization(double input) {
        io.runCharacterization(input);
    }

    public SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                // Gaslight SysId since motor is actually running amps instead of volts, feedforwards should still be accurate
                Volts.of(0.5).per(Second),
                null, 
                null,
                (state) -> Logger.recordOutput("ClimbSysIdState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.runCharacterization(voltage.in(Volts)), 
                null, 
                this
            )
        );
    }

    public Command sysIdQuasistaticForward() {
        return getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdQuasistaticReverse() {
        return getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdDynamicForward() {
        return getSysIdRoutine().dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamicReverse() {
        return getSysIdRoutine().dynamic(SysIdRoutine.Direction.kReverse);
    }

}
