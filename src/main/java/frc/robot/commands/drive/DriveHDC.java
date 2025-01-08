package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.managers.HDCTuner;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;

public class DriveHDC extends Command {

    private final Swerve swerve;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier shouldMirror;

    private Pose2d desiredPose = new Pose2d();

    public DriveHDC (
            Swerve swerve,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationsSupplier,
            BooleanSupplier fieldRelativeSupplier,
            BooleanSupplier shouldMirror,
            HDCTuner HDCCalibration) {

        this.swerve = swerve;

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationsSupplier;

        this.shouldMirror = shouldMirror;

        addRequirements(swerve);
    }

    public DriveHDC(Swerve swerve, Supplier<ChassisSpeeds> speeds, BooleanSupplier fieldRelativeSupplier, BooleanSupplier shouldMirror) {

        this.swerve = swerve;

        this.xSupplier = () -> speeds.get().vxMetersPerSecond;
        this.ySupplier = () -> speeds.get().vyMetersPerSecond;
        this.rotationSupplier = () -> speeds.get().omegaRadiansPerSecond;
        this.shouldMirror = shouldMirror;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double x = xSupplier.getAsDouble();
        // The driver's right is negative 
        // on the field's axis
        double y = -ySupplier.getAsDouble();
        if (shouldMirror.getAsBoolean()) {
            x *= -1;
            y *= -1;
        }

        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotationSupplier.getAsDouble(), swerve.getPose().getRotation());

        // If the desired pose is more than 2 meters away, reset it to the current pose
        // This is to prevent the robot from chasing the sun
        if (desiredPose.getTranslation().getDistance(swerve.getPose().getTranslation()) > 2) {
            desiredPose = swerve.getPose();
        }

        // integrate the speeds to positions with exp and twist
        desiredPose = desiredPose.exp(
            new Twist2d(
                desiredSpeeds.vxMetersPerSecond * DriveConstants.MAX_SPEED_METERS_PER_SECOND * .02,
                desiredSpeeds.vyMetersPerSecond * DriveConstants.MAX_SPEED_METERS_PER_SECOND * .02,
                desiredSpeeds.omegaRadiansPerSecond * DriveConstants.MAX_SPEED_METERS_PER_SECOND * .015
            )
        );


        swerve.drive(
            AutoConstants.HDC.calculate(swerve.getPose(), desiredPose, 0, desiredPose.getRotation())
        );

        swerve.setDesiredPose(desiredPose);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
