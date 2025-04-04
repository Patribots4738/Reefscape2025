// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.RobotContainer;
import frc.robot.util.Constants.WristConstants;
import frc.robot.util.Constants.LoggingConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Wrist extends SubsystemBase {

    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private State targetState = new State(0, 0);
    private State setpoint = new State(0, 0);
    @AutoLogOutput (key = "Subsystems/Wrist/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;
    @AutoLogOutput (key = "Subsystems/Wrist/ShouldRunFastProfile")
    private boolean shouldRunFast = true;

    private TrapezoidProfile fastProfile = new TrapezoidProfile(new Constraints(WristConstants.FAST_VELOCITY, WristConstants.FAST_ACCELERATION));
    private TrapezoidProfile slowProfile = new TrapezoidProfile(new Constraints(WristConstants.SLOW_VELOCITY, WristConstants.SLOW_ACCELERATION));

    // private LoggedTunableNumber fastVelocity = new LoggedTunableNumber("Wrist/FastProfile/Velocity", WristConstants.FAST_VELOCITY);
    // private LoggedTunableNumber fastAcceleration = new LoggedTunableNumber("Wrist/FastProfile/Acceleration", WristConstants.FAST_ACCELERATION);
    // private LoggedTunableNumber slowVelocity = new LoggedTunableNumber("Wrist/SlowProfile/Velocity", WristConstants.SLOW_VELOCITY);
    // private LoggedTunableNumber slowAcceleration = new LoggedTunableNumber("Wrist/SlowProfile/Acceleration", WristConstants.SLOW_ACCELERATION);

    public Wrist(WristIO io) {
        this.io = io;

        // fastVelocity.onChanged().or(fastAcceleration.onChanged()).onTrue(Commands.runOnce(() -> fastProfile = new TrapezoidProfile(new Constraints(fastVelocity.get(), fastAcceleration.get()))).ignoringDisable(true));
        // slowVelocity.onChanged().or(slowAcceleration.onChanged()).onTrue(Commands.runOnce(() -> slowProfile = new TrapezoidProfile(new Constraints(slowVelocity.get(), slowAcceleration.get()))).ignoringDisable(true));

        RobotContainer.desiredComponents3d[LoggingConstants.WRIST_INDEX] = new Pose3d(
            LoggingConstants.WRIST_OFFSET.getX(),
            LoggingConstants.WRIST_OFFSET.getY(),
            LoggingConstants.WRIST_OFFSET.getZ(),
            RobotContainer.desiredComponents3d[LoggingConstants.WRIST_INDEX].getRotation()
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Wrist", inputs);

        Logger.recordOutput("Subsystems/Wrist/AtTargetPosition", atTargetPosition());
        Logger.recordOutput("Subsystems/Wrist/TargetPosition", targetState.position);

        if (shouldRunSetpoint) {
            setpoint = (shouldRunFast ? fastProfile : slowProfile).calculate(0.02, setpoint, targetState);
            Logger.recordOutput("Subsystems/Wrist/Step/Velocity", setpoint.velocity);
            Logger.recordOutput("Subsystems/Wrist/Step/Position", setpoint.position);
            io.setPosition(setpoint.position, 0);
        } else {
            setpoint.position = inputs.internalPositionRads;
            setpoint.velocity = inputs.internalVelocityRadsPerSec;
            io.setNeutral();
        }

        RobotContainer.components3d[LoggingConstants.WRIST_INDEX] = new Pose3d(
            RobotContainer.components3d[LoggingConstants.WRIST_INDEX].getX(), 
            RobotContainer.components3d[LoggingConstants.WRIST_INDEX].getY(),
            RobotContainer.components3d[LoggingConstants.WRIST_INDEX].getZ(),
            new Rotation3d(0, inputs.internalPositionRads, 0)
        );

    }

    public void setPosition(double position, boolean runFast) {
        position = MathUtil.clamp(position, WristConstants.MIN_ANGLE_RADIANS, WristConstants.MAX_ANGLE_RADIANS);
        targetState.position = position;
        setpoint.position = inputs.internalPositionRads;
        setpoint.velocity = 0;
        shouldRunSetpoint = true;
        shouldRunFast = runFast;

        RobotContainer.desiredComponents3d[LoggingConstants.WRIST_INDEX] = new Pose3d(
            RobotContainer.desiredComponents3d[LoggingConstants.WRIST_INDEX].getX(), 
            RobotContainer.desiredComponents3d[LoggingConstants.WRIST_INDEX].getY(),
            RobotContainer.desiredComponents3d[LoggingConstants.WRIST_INDEX].getZ(),
            new Rotation3d(0, position, 0)
        );
    }

    public void setNeutral() {
        shouldRunSetpoint = false;
    }

    public Command setPositionCommand(DoubleSupplier positionSupplier, BooleanSupplier runFastSupplier) {
        return runOnce(() -> setPosition(positionSupplier.getAsDouble(), runFastSupplier.getAsBoolean())).andThen(Commands.waitUntil(this::atTargetPosition));
    }

    public Command setPositionCommand(double position, BooleanSupplier runFastSupplier) {
        return setPositionCommand(() -> position, runFastSupplier);
    }
    
    public Command setNeutralCommand() {
        return runOnce(this::setNeutral);
    }

    public boolean atPosition(double position) {
        return MathUtil.isNear(position, inputs.internalPositionRads, WristConstants.POSITION_DEADBAND_RADIANS);
    }

    public boolean atTargetPosition() {
        return atPosition(targetState.position);
    }

    public double getPosition() {
        return inputs.internalPositionRads;
    }

    public double getVelocity() {
        return inputs.internalVelocityRadsPerSec;
    }

    public double getTargetPosition() {
        return targetState.position;
    }

    public double getCharacterizationVelocity() {
        return inputs.internalVelocityRadsPerSec / WristConstants.VELOCITY_CONVERSION_FACTOR;
    }

    public void runCharacterization(double input) {
        io.runCharacterization(input);
    }

    public SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                // Gaslight SysId since motor is actually running amps instead of volts, feedforwards should still be accurate
                Volts.of(1).per(Second),
                null, 
                null,
                (state) -> Logger.recordOutput("WristSysIdState", state.toString())
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
