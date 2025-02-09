// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import frc.robot.RobotContainer;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.Constants.LoggingConstants;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import frc.robot.util.custom.LoggedTunableBoolean;
import frc.robot.util.custom.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    
    private final LoggedTunableBoolean brakeMotor = new LoggedTunableBoolean("Elevator/BrakeMotor", ElevatorConstants.BRAKE_MOTOR);
    private final LoggedTunableNumber velocity = new LoggedTunableNumber("Elevator/Profile/Velocity", ElevatorConstants.VELOCITY);
    private final LoggedTunableNumber acceleration = new LoggedTunableNumber("Elevator/Profile/Acceleration", ElevatorConstants.ACCELERATION);
    private final LoggedTunableNumber jerk = new LoggedTunableNumber("Elevator/Profile/Jerk", ElevatorConstants.JERK);

    private double targetPosition = 0.0;
    
    public Elevator(ElevatorIO io) {
        this.io = io;
        brakeMotor.onChanged(runOnce(() -> this.io.setBrakeMode(brakeMotor.get())).ignoringDisable(true));
        velocity.onChanged().or(acceleration.onChanged()).or(jerk.onChanged()).onTrue(runOnce(() -> io.configureProfile(velocity.get(), acceleration.get(), jerk.get())).ignoringDisable(true));
        ElevatorConstants.LOGGED_GAINS.onChanged(runOnce(() -> io.setGains(ElevatorConstants.LOGGED_GAINS.get())).ignoringDisable(true));
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
        io.setPosition(targetPosition);

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

    public Command resetEncodersCommand() {
        return runOnce(() -> io.resetEncoders(0));
    }

    public boolean atPosition(double position) {
        return MathUtil.isNear(position, inputs.leaderPositionMeters, ElevatorConstants.DEADBAND_METERS);
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

    public SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                    // Gaslight SysId since motor is actually running amps instead of volts, feedforwards should still be accurate
                    Volts.of(0.1).per(Second),
                    null, 
                    null,
                    (state) -> Logger.recordOutput("ElevatorSysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    public Command sysIdQuasistatic() {
        return getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamic() {
        return getSysIdRoutine().dynamic(SysIdRoutine.Direction.kForward);
    }
}
