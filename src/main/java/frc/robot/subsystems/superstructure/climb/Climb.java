// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.climb;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

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
import frc.robot.util.custom.LoggedTunableBoolean;
import frc.robot.util.custom.LoggedTunableNumber;

public class Climb extends SubsystemBase {
    
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private final LoggedTunableBoolean brakeMotor = new LoggedTunableBoolean("Climb/BrakeMotor", ClimbConstants.BRAKE_MOTOR);
    private final LoggedTunableNumber velocity = new LoggedTunableNumber("Climb/Profile/Velocity", ClimbConstants.VELOCITY);
    private final LoggedTunableNumber acceleration = new LoggedTunableNumber("Climb/Profile/Acceleration", ClimbConstants.ACCELERATION);
    private final LoggedTunableNumber jerk = new LoggedTunableNumber("Climb/Profile/Jerk", ClimbConstants.JERK);

    private final LoggedTunableNumber stowPosition = new LoggedTunableNumber("Climb/StowPosition", ClimbConstants.STOW_POSITION_RADIANS);
    private final LoggedTunableNumber readyPosition = new LoggedTunableNumber("Climb/ReadyPosition", ClimbConstants.READY_POSITION_RADIANS);
    private final LoggedTunableNumber finalPosition = new LoggedTunableNumber("Climb/FinalPosition", ClimbConstants.FINAL_POSITION_RADIANS);

    private double targetPosition = 0.0;

    public Climb(ClimbIO io) {
        this.io = io;
        brakeMotor.onChanged(runOnce(() -> this.io.setBrakeMode(brakeMotor.get())).ignoringDisable(true));
        velocity.onChanged().or(acceleration.onChanged()).or(jerk.onChanged()).onTrue(runOnce(() -> io.configureProfile(velocity.get(), acceleration.get(), jerk.get())).ignoringDisable(true));
        ClimbConstants.LOGGED_GAINS.onChanged(runOnce(() -> io.setGains(ClimbConstants.LOGGED_GAINS.get())).ignoringDisable(true));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Climb", inputs);
        Logger.recordOutput("Subsystems/Climb/AtDesiredPosition", atTargetPosition());

        RobotContainer.components3d[LoggingConstants.CLIMB_INDEX] = new Pose3d(
            LoggingConstants.CLIMB_OFFSET, 
            new Rotation3d(-inputs.leaderPositionRads, 0, 0)
        );
    }

    public void setPosition(double position) {
        position = MathUtil.clamp(position, ClimbConstants.MIN_ANGLE_RADIANS, ClimbConstants.MAX_ANGLE_RADIANS);
        targetPosition = position;
        io.setPosition(targetPosition);

        RobotContainer.desiredComponents3d[LoggingConstants.CLIMB_INDEX] = new Pose3d(
            LoggingConstants.CLIMB_OFFSET,
            new Rotation3d(-position, 0, 0)
        );
    }

    public void setNeutral() {
        io.setNeutral();
    }

    public Command setPositionCommand(DoubleSupplier positionSupplier) {
        return runOnce(() -> setPosition(positionSupplier.getAsDouble())).andThen(Commands.waitUntil(this::atTargetPosition));
    }

    public Command setPositionCommand(double position) {
        return setPositionCommand(() -> position);
    }

    public Command setNeutralCommand() {
        return runOnce(this::setNeutral);
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

    public boolean atPosition(double position) {
        return MathUtil.isNear(position, inputs.leaderPositionRads, ClimbConstants.DEADBAND_RADIANS);
    }

    public boolean atTargetPosition() {
        return atPosition(targetPosition);
    }

    public double getPosition() {
        return inputs.leaderPositionRads;
    }

    public double getCharacterizationVelocity() {
        return inputs.leaderVelocityRadsPerSec / ClimbConstants.VELOCITY_CONVERSION_FACTOR;
    }

    public void runCharacterization(double input) {
        io.runCharacterization(input);
    }

    public SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                // Gaslight SysId since motor is actually running amps instead of volts, feedforwards should still be accurate
                Volts.of(0.1).per(Second),
                null, 
                Seconds.of(2),
                (state) -> Logger.recordOutput("WristSysIdState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.runCharacterization(voltage.in(Volts)), 
                null, 
                this
            )
        );
    }

    public Command sysIdQuasistatic() {
        return getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamic() {
        return getSysIdRoutine().dynamic(SysIdRoutine.Direction.kForward);
    }

}
