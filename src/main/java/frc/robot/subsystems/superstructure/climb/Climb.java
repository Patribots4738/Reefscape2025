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
import frc.robot.util.custom.LoggedTunableBoolean;
import frc.robot.util.custom.LoggedTunableNumber;

public class Climb extends SubsystemBase {
    
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private final LoggedTunableBoolean brakeMotor = new LoggedTunableBoolean("Climb/BrakeMotor", ClimbConstants.BRAKE_MOTOR);
    private final LoggedTunableNumber velocity = new LoggedTunableNumber("Climb/Profile/Velocity", ClimbConstants.VELOCITY);
    private final LoggedTunableNumber acceleration = new LoggedTunableNumber("Climb/Profile/Acceleration", ClimbConstants.ACCELERATION);
    private final LoggedTunableNumber jerk = new LoggedTunableNumber("Climb/Profile/Jerk", ClimbConstants.JERK);

    private double targetPosition = 0.0;

    public Climb(ClimbIO io) {
        this.io = io;
        brakeMotor.onChanged(runOnce(() -> this.io.setBrakeMode(brakeMotor.get())).ignoringDisable(true));
        velocity.onChanged().or(acceleration.onChanged()).or(jerk.onChanged()).onTrue(runOnce(() -> io.configureProfile(velocity.get(), acceleration.get(), jerk.get())).ignoringDisable(true));
        ClimbConstants.LOGGED_SLOW_GAINS.onChanged(runOnce(() -> io.setGains(ClimbConstants.LOGGED_SLOW_GAINS.get(), 0)).ignoringDisable(true));
        ClimbConstants.LOGGED_FAST_GAINS.onChanged(runOnce(() -> io.setGains(ClimbConstants.LOGGED_FAST_GAINS.get(), 1)).ignoringDisable(true));
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
