// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import frc.robot.RobotContainer;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.Constants.LoggingConstants;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private State targetState = new State(0, 0);
    private State setpoint = new State(0, 0);
    @AutoLogOutput (key = "Subsystems/Elevator/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;
    @AutoLogOutput (key = "Subsystems/Elevator/ShouldRunFastProfile")
    private boolean shouldRunFast = true;

    private TrapezoidProfile fastProfile = new TrapezoidProfile(new Constraints(ElevatorConstants.FAST_VELOCITY, ElevatorConstants.FAST_ACCELERATION));
    private TrapezoidProfile slowProfile = new TrapezoidProfile(new Constraints(ElevatorConstants.SLOW_VELOCITY, ElevatorConstants.SLOW_ACCELERATION));

    // private LoggedTunableNumber fastVelocity = new LoggedTunableNumber("Elevator/FastProfile/Velocity", ElevatorConstants.FAST_VELOCITY);
    // private LoggedTunableNumber fastAcceleration = new LoggedTunableNumber("Elevator/FastProfile/Acceleration", ElevatorConstants.FAST_ACCELERATION);
    // private LoggedTunableNumber slowVelocity = new LoggedTunableNumber("Elevator/SlowProfile/Velocity", ElevatorConstants.SLOW_VELOCITY);
    // private LoggedTunableNumber slowAcceleration = new LoggedTunableNumber("Elevator/SlowProfile/Acceleration", ElevatorConstants.SLOW_ACCELERATION);

    // private final LoggedTunableBoolean brakeMotor = new LoggedTunableBoolean("Elevator/BrakeMotor", ElevatorConstants.BRAKE_MOTOR);
    
    public Elevator(ElevatorIO io) {
        this.io = io;

        // fastVelocity.onChanged().or(fastAcceleration.onChanged()).onTrue(Commands.runOnce(() -> fastProfile = new TrapezoidProfile(new Constraints(fastVelocity.get(), fastAcceleration.get()))).ignoringDisable(true));
        // slowVelocity.onChanged().or(slowAcceleration.onChanged()).onTrue(Commands.runOnce(() -> slowProfile = new TrapezoidProfile(new Constraints(slowVelocity.get(), slowAcceleration.get()))).ignoringDisable(true));

        // ElevatorConstants.LOGGED_GAINS.onChanged(Commands.runOnce(() -> io.setGains(ElevatorConstants.LOGGED_GAINS.get())).ignoringDisable(true));
        // brakeMotor.onChanged(Commands.runOnce(() -> io.setBrakeMode(brakeMotor.get())).ignoringDisable(true));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Elevator", inputs);

        Logger.recordOutput("Subsystems/Elevator/AtTargetPosition", atTargetPosition());
        Logger.recordOutput("Subsystems/Elevator/TargetPosition", targetState.position);

        if (shouldRunSetpoint) {
            setpoint = (shouldRunFast ? fastProfile : slowProfile).calculate(0.02, setpoint, targetState);
            Logger.recordOutput("Subsystems/Elevator/Step/Velocity", setpoint.velocity);
            Logger.recordOutput("Subsystems/Elevator/Step/Position", setpoint.position);
            io.setPosition(setpoint.position);
        } else {
            setpoint.position = inputs.leaderPositionMeters;
            setpoint.velocity = inputs.leaderVelocityMetersPerSecond;
            io.setNeutral();
        }

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

    public void setPosition(double position, boolean runFast) {
        position = MathUtil.clamp(position, 0, ElevatorConstants.MAX_DISPLACEMENT_METERS);
        targetState.position = position;
        shouldRunSetpoint = true;
        shouldRunFast = runFast;

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

    public Command resetEncodersCommand() {
        return runOnce(() -> io.resetEncoders(0));
    }

    public boolean atPosition(double position) {
        return MathUtil.isNear(position, inputs.leaderPositionMeters, ElevatorConstants.DEADBAND_METERS) || MathUtil.isNear(position, inputs.followerPositionMeters, ElevatorConstants.DEADBAND_METERS);
    }

    public boolean atTargetPosition() {
        return atPosition(targetState.position);
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
                Volts.of(0.5).per(Second),
                null, 
                Seconds.of(2),
                (state) -> Logger.recordOutput("ElevatorSysIdState", state.toString())
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
