package frc.robot.commands.drive;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.commands.managers.HDCTuner;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;

public class DriveCurvedHDC extends Command {
    private final Swerve swerve;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier shouldMirror;
    private final Rotation2d desiredHeading;
    private final List<Translation2d> waypoints;

    private Pose2d endPose;
    
    public DriveCurvedHDC (
            Swerve swerve,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationsSupplier,
            BooleanSupplier fieldRelativeSupplier,
            BooleanSupplier shouldMirror,
            Rotation2d desiredHeading,
            Pose2d endPose,
            List<Translation2d> interiorWaypoints,
            HDCTuner HDCCalibration) {

        this.swerve = swerve;

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationsSupplier;
        this.endPose = endPose;
        this.desiredHeading = desiredHeading;

        this.waypoints = interiorWaypoints;

        this.shouldMirror = shouldMirror;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // create a list of points along a curved trajectory
        List<Trajectory.State> points = TrajectoryGenerator.generateTrajectory(swerve.getPose(), waypoints, endPose, AutoConstants.REEF_ALIGNMENT_TRAJECTORY_CONFIG).getStates();

        // If the desired pose is more than 2 meters away, reset it to the current pose
        // This is to prevent the robot from chasing the sun
        if (endPose.getTranslation().getDistance(swerve.getPose().getTranslation()) > 2) {
            endPose = swerve.getPose();
        }

        // move to each point along the curve
        for (Trajectory.State point : points) {
            swerve.drive(
                AutoConstants.TELE_HDC.calculate(swerve.getPose(), point, desiredHeading)
            );
        }

        swerve.setDesiredPose(endPose);
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