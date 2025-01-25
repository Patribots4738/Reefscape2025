package frc.robot.util.auto;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.custom.ReefSide;

public class Alignment {

    private final Swerve swerve;
    private AlignmentMode mode = AlignmentMode.NONE;
    private int alignmentIndex = -1;

    public Alignment(Swerve swerve) {
        this.swerve = swerve;
    }

    public enum AlignmentMode {
        INTAKE,
        CAGE,
        REEF,
        NONE
    }

    public void resetHDC() {
        // Reset error and integral terms of HDC's PID controller
        AutoConstants.TELE_HDC.getThetaController().reset(swerve.getPose().getRotation().getRadians());
        AutoConstants.TELE_HDC.getXController().reset();
        AutoConstants.TELE_HDC.getYController().reset();
    }

    public Command resetHDCCommand() {
        return Commands.runOnce(() -> resetHDC());
    }

    public double normalizeTwoSpeeds(double controllerInput, double autoInput) {
        // Normalize HDC input between -1 and 1
        autoInput /= swerve.getMaxLinearVelocity();

        // Combine speeds and normalize them between -1 and 1
        return MathUtil.clamp(
            controllerInput + autoInput,
            -1,
            1
        );
    }
    
    public ChassisSpeeds normalizeChassisSpeeds(ChassisSpeeds autoSpeeds, ChassisSpeeds controllerSpeeds) {
        return new ChassisSpeeds(
            MathUtil.applyDeadband((normalizeTwoSpeeds(controllerSpeeds.vyMetersPerSecond, autoSpeeds.vyMetersPerSecond)), 0.001),
            -MathUtil.applyDeadband((normalizeTwoSpeeds(controllerSpeeds.vxMetersPerSecond, autoSpeeds.vxMetersPerSecond)), 0.001),
            MathUtil.applyDeadband(autoSpeeds.omegaRadiansPerSecond / swerve.getMaxAngularVelocity(), 0.001)
        );
    }

    public ChassisSpeeds getAutoSpeeds(Pose2d position) {
        swerve.setDesiredPose(position);
        return AutoConstants.TELE_HDC.calculate(swerve.getPose(), position, 0, position.getRotation());
    }

    public ChassisSpeeds getAutoRotationalSpeeds(Rotation2d rotation) {
        Pose2d desiredPose = new Pose2d(swerve.getPose().getX(), swerve.getPose().getY(), rotation);
        swerve.setDesiredPose(desiredPose);
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
                swerve.getPose().getRotation()
            );
    }

    public ChassisSpeeds getIntakeAutoSpeeds() {
        Rotation2d intakeRotation = swerve.getPose().nearest(FieldConstants.GET_CORAL_STATION_POSITIONS()).getRotation();
        return getAutoRotationalSpeeds(intakeRotation);
    }

    public ChassisSpeeds getCageAutoSpeeds() {
        Pose2d cagePose;
        if (alignmentIndex == -1) {
            // First alignment, use nearest pose and set alignmentIndex
            int index = PoseCalculations.nearestIndex(swerve.getPose(), FieldConstants.GET_CAGE_POSITIONS());
            cagePose = FieldConstants.GET_CAGE_POSITIONS().get(index);
            alignmentIndex = index;
        } else {
            // Post-alignment, use current alignmentIndex
            cagePose = FieldConstants.GET_CAGE_POSITIONS().get(alignmentIndex);
        }
        Pose2d desiredPose = new Pose2d(swerve.getPose().getX(), cagePose.getY(), cagePose.getRotation());
        return getAutoSpeeds(desiredPose);
    }

    public ChassisSpeeds getReefAutoSpeeds() {
        Pose2d currentPose = swerve.getPose();
        ReefSide reefSide = PoseCalculations.getClosestReefSide(swerve.getPose());
        Pose2d left = reefSide.getLeft();
        Pose2d right = reefSide.getRight();
        Pose2d node;
        if (alignmentIndex == -1) {
            // First alignment, find closest reef node
            double leftDist = currentPose.getTranslation().getDistance(left.getTranslation());
            double rightDist = currentPose.getTranslation().getDistance(right.getTranslation());
            boolean leftCloser = leftDist < rightDist;
            node = leftCloser ? left : right;
            alignmentIndex = leftCloser ? 0 : 1;
        } else {
            // Post-alignment, use current alignmentIndex
            node = alignmentIndex == 0 ? left : right;
        }
        // Get distance from reef center to reef face center
        Pose2d relativeCenter = reefSide.getCenter().relativeTo(FieldConstants.GET_REEF_POSITION());
        // "Recenter" alignment off of the left or right node
        Pose2d centerPose = 
            new Pose2d(
                node.getX() - (relativeCenter.getX() * (Robot.isRedAlliance() ? -1 : 1)), 
                node.getY() - relativeCenter.getY() * (Robot.isRedAlliance() ? -1 : 1), 
                node.getRotation());
        double distance = currentPose.getTranslation().getDistance(centerPose.getTranslation());
        // Get desired position from face angle
        double x = centerPose.getX() + distance * node.getRotation().getCos();
        double y = centerPose.getY() + distance * node.getRotation().getSin();
        Pose2d desiredPose = new Pose2d(x, y, node.getRotation());
        return getAutoSpeeds(desiredPose);
    }

    public ChassisSpeeds getReefControllerSpeeds(double driverX, double driverY) {
        ReefSide reefFace = PoseCalculations.getClosestReefSide(swerve.getPose());
        // Use either driverX or driverY axis depending on reef face (driverY faces are on the Y center of the field)
        double axis = 
            (reefFace.getCenter().getY() == FieldConstants.FIELD_MAX_HEIGHT / 2.0 
                ? driverY 
                : driverX) * AutoConstants.REEF_ALIGNMENT_MULTIPLIER;
        double reefX = reefFace.getCenter().getX();
        double reefY = reefFace.getCenter().getY();
        Pose2d reefPosition = FieldConstants.GET_REEF_POSITION();
        // Invert axis depending on reef face
        if ((reefY == FieldConstants.FIELD_MAX_HEIGHT / 2.0 
            && (Robot.isRedAlliance() 
                ? reefX < reefPosition.getX() 
                : reefX > reefPosition.getX())) 
            || (Robot.isRedAlliance() 
                ? reefY > reefPosition.getY() 
                : reefY < FieldConstants.FIELD_MAX_HEIGHT / 2.0)) 
        {
            axis *= -1;
        }
        // Get velocity vector components via face angle
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            -axis * reefFace.getRotation().getCos(),
            -axis * reefFace.getRotation().getSin(),
            0,
            swerve.getPose().getRotation()
        );
        return speeds;
    }

    public void updateIndex(int increment) {
        alignmentIndex += increment;
        // Clamp index based on number of alignment poses in the current mode
        alignmentIndex = 
            switch (mode) {
                case CAGE -> MathUtil.clamp(alignmentIndex, 0, FieldConstants.GET_CAGE_POSITIONS().size() - 1);
                case REEF -> MathUtil.clamp(alignmentIndex, 0, 1);
                default -> alignmentIndex = -1;
            };
    }

    public Command updateIndexCommand(int increment) {
        return Commands.runOnce(() -> updateIndex(increment));
    }

    public Command autoAlignmentCommand(AlignmentMode mode, Supplier<ChassisSpeeds> autoSpeeds, Supplier<ChassisSpeeds> controllerSpeeds) {
        return 
            Commands.sequence(
                Commands.runOnce(() -> this.mode = mode),
                swerve.getDriveCommand(
                () -> 
                    normalizeChassisSpeeds(
                        autoSpeeds.get(), 
                        controllerSpeeds.get()
                    ), 
                    () -> false
                )
            ).finallyDo(() -> {
                resetHDC();
                swerve.setDesiredPose(new Pose2d());
                this.mode = AlignmentMode.NONE;
                this.alignmentIndex = -1;
            });
    }

    public Command intakeAlignmentCommand(DoubleSupplier driverX, DoubleSupplier driverY) {
        return 
            autoAlignmentCommand(
                AlignmentMode.INTAKE, 
                this::getIntakeAutoSpeeds, 
                () -> getControllerSpeeds(driverX.getAsDouble(), 
                driverY.getAsDouble()));
    }

    public Command cageAlignmentCommand(DoubleSupplier driverY) {
        return 
            autoAlignmentCommand(
                AlignmentMode.CAGE, 
                this::getCageAutoSpeeds, 
                () -> getControllerSpeeds(0, driverY.getAsDouble() * AutoConstants.CAGE_ALIGNMENT_MULTIPLIER));
    }

    public Command reefAlignmentCommand(DoubleSupplier driverX, DoubleSupplier driverY) {
        return 
            autoAlignmentCommand(
                AlignmentMode.REEF, 
                this::getReefAutoSpeeds, 
                () -> getReefControllerSpeeds(driverX.getAsDouble(), driverY.getAsDouble()));
    }

}
