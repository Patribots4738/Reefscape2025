package frc.robot.util.auto;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.FieldConstants;

public class Alignment {

    private final Swerve swerve;

    public Alignment(Swerve swerve) {
        this.swerve = swerve;
    }

    public double normalizeTwoSpeeds(double controllerInput, double autoInput) {
        autoInput /= swerve.getMaxLinearVelocity();

        return MathUtil.clamp(
            controllerInput + autoInput,
            -1,
            1);
    }
    
    public ChassisSpeeds normalizeChassisSpeeds(ChassisSpeeds autoSpeeds, ChassisSpeeds controllerSpeeds) {
        return new ChassisSpeeds(
            normalizeTwoSpeeds(controllerSpeeds.vyMetersPerSecond, autoSpeeds.vyMetersPerSecond),
            -normalizeTwoSpeeds(controllerSpeeds.vxMetersPerSecond, autoSpeeds.vxMetersPerSecond),
            autoSpeeds.omegaRadiansPerSecond / swerve.getMaxAngularVelocity()
        );
    }

    public ChassisSpeeds getAutoSpeeds(Pose2d position) {
        return AutoConstants.TELE_HDC.calculate(swerve.getPose(), position, 0, position.getRotation());
    }

    public ChassisSpeeds getAutoRotationalSpeeds(Rotation2d rotation) {
        return 
            new ChassisSpeeds(
                0,
                0,
                AutoConstants.TELE_HDC.getThetaController().calculate(swerve.getPose().getRotation().getRadians(), rotation.getRadians())
            );
    }

    public ChassisSpeeds getControllerSpeeds(double controllerX, double controllerY) {
        return
            ChassisSpeeds.fromFieldRelativeSpeeds(
                controllerY * (Robot.isRedAlliance() ? -1 : 1), 
                controllerX * (Robot.isRedAlliance() ? 1 : -1), 
                0, 
                swerve.getPose().getRotation());
    }

    public ChassisSpeeds getIntakeRotationalSpeeds() {
        Rotation2d intakeRotation = swerve.getPose().nearest(FieldConstants.GET_CORAL_STATION_POSITIONS()).getRotation();
        return getAutoRotationalSpeeds(intakeRotation);
    }

    public Command autoAlignmentCommand(Supplier<ChassisSpeeds> autoSpeeds, Supplier<ChassisSpeeds> controllerSpeeds) {
        return 
            swerve.getDriveCommand(
                () -> 
                    normalizeChassisSpeeds(
                        autoSpeeds.get(), 
                        controllerSpeeds.get()
                    ), 
                () -> false
            );
    }

    public Command intakeAlignmentCommand(DoubleSupplier driverX, DoubleSupplier driverY) {
        return autoAlignmentCommand(this::getIntakeRotationalSpeeds, () -> getControllerSpeeds(driverX.getAsDouble(), driverY.getAsDouble()));
    }

}
