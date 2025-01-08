// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.characterization;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.Constants.DriveConstants;
import java.util.function.DoubleSupplier;

// thanks 6328 :D <3
public class WheelRadiusCharacterization extends Command {
    // wheel radius (meters) =
    // gyro delta (radians) * drive base radius (meters) / wheel position delta
    // (radians)
    private double currentEffectiveWheelRadius = 0.0;

    private double lastGyroYawRads = 0.0, accumGyroYawRads = 0.0;

    private double[] startWheelPositions;

    private Swerve swerve;
    private final DoubleSupplier gyroYawRadsSupplier = () -> swerve.getGyroRotation2d().getRadians();

    // wheel radius (meters) =
    // gyro delta (radians) * drive base radius (meters) / wheel position delta
    // (radians)
    public WheelRadiusCharacterization(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        accumGyroYawRads = 0.0;

        startWheelPositions = swerve.getWheelRadiusCharacterizationPosition();

        System.out.println("\nStarted wheel radius characterization\n");
    }

    @Override
    public void execute() {
        
        swerve.drive(new ChassisSpeeds(0.0, 0.0, .5));
        
        accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        
        double averageWheelPosition = 0.0;
        double[] wheelPositions = swerve.getWheelRadiusCharacterizationPosition();
        
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
        }

        System.out.println(averageWheelPosition);

        averageWheelPosition /= 4.0;
        
        double driveRadius = Math.hypot(DriveConstants.WHEEL_BASE, DriveConstants.TRACK_WIDTH) / 2.0;
        
        currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopDriving();
        if (accumGyroYawRads <= Math.PI * 2.0) {
            System.out.println("\nNot enough data for characterization\n" + accumGyroYawRads);
        } else {
            System.out.println(
                "\nEffective Wheel Radius: "
                    + Units.metersToInches(currentEffectiveWheelRadius)
                    + " inches\n");
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return accumGyroYawRads > Math.PI * 2;
    }
}
