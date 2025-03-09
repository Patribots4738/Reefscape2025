package frc.robot.util.auto;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.Constants.AlgaeClawConstants;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.custom.LoggedTunableNumber;
import frc.robot.util.custom.ReefSide;

// create motion profile DONE
// create tunnable motion profile DONE
// use setpoints and target state to run motion profile in command?

public class Alignment {

    private final Swerve swerve;
    @AutoLogOutput (key = "Subsystems/Swerve/AlignmentMode")
    private AlignmentMode alignmentMode = AlignmentMode.NONE;
    @AutoLogOutput (key = "Subsystems/Swerve/AlignmentIndex")
    private int alignmentIndex = -1;

    private TrapezoidProfile translationProfile = new TrapezoidProfile(new Constraints(AutoConstants.HDC_XY_VELOCITY, AutoConstants.HDC_XY_ACCELERATION));
    private TrapezoidProfile rotationProfile = new TrapezoidProfile(new Constraints(AutoConstants.HDC_THETA_VELOCITY, AutoConstants.HDC_THETA_ACCELERATION));

    private State xSetpoint = new State(0, 0);
    private State ySetpoint = new State(0, 0);
    private State targetX = new State(0, 0);
    private State targetY = new State(0, 0);

    private LoggedTunableNumber translationVelocity = new LoggedTunableNumber("Alignment/TranslationVelocity", AutoConstants.HDC_XY_VELOCITY);
    private LoggedTunableNumber translationAcceleration = new LoggedTunableNumber("Alignment/TranslationAcceleration", AutoConstants.HDC_XY_ACCELERATION);
    private LoggedTunableNumber rotationVelocity = new LoggedTunableNumber("Alignment/RotationVelocity", AutoConstants.HDC_THETA_VELOCITY);
    private LoggedTunableNumber rotationAcceleration = new LoggedTunableNumber("Alignment/RotationAcceleration", AutoConstants.HDC_THETA_ACCELERATION);
    
    public Alignment(Swerve swerve) {
        this.swerve = swerve;

        rotationVelocity.onChanged().or(rotationAcceleration.onChanged()).onTrue(Commands.runOnce(() -> rotationProfile = new TrapezoidProfile(new Constraints(rotationVelocity.get(), rotationAcceleration.get()))).ignoringDisable(true));
        translationVelocity.onChanged().or(translationAcceleration.onChanged()).onTrue(Commands.runOnce(() -> translationProfile = new TrapezoidProfile(new Constraints(translationVelocity.get(), translationAcceleration.get()))).ignoringDisable(true));
    }

    public enum AlignmentMode {
        INTAKE,
        INTAKE_SOFT,
        CAGE,
        REEF,
        REEF_SOFT,
        NET,
        NONE
    }

    public void resetHDC() {
        // Reset error and integral terms of HDC's PID controller
        AutoConstants.TELE_HDC.getThetaController().reset(swerve.getPose().getRotation().getRadians());
        AutoConstants.TELE_HDC.getXController().reset();
        AutoConstants.TELE_HDC.getYController().reset();
        xSetpoint.position = 0;
        ySetpoint.position = 0;
        targetX.position = 0;
        targetY.position = 0;
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

    public double normalizeSpeed(double autoInput) {
        // Normalize HDC input between -1 and 1
        autoInput /= swerve.getMaxLinearVelocity();

        // Combine speeds and normalize them between -1 and 1
        return MathUtil.clamp(
            autoInput,
            -1,
            1
        );
    }
    
    public ChassisSpeeds normalizeChassisSpeeds(ChassisSpeeds autoSpeeds, ChassisSpeeds controllerSpeeds) {
        return new ChassisSpeeds(
            MathUtil.applyDeadband((normalizeTwoSpeeds(controllerSpeeds.vyMetersPerSecond, autoSpeeds.vyMetersPerSecond)), 0.01),
            -MathUtil.applyDeadband((normalizeTwoSpeeds(controllerSpeeds.vxMetersPerSecond, autoSpeeds.vxMetersPerSecond)), 0.01),
            MathUtil.applyDeadband(autoSpeeds.omegaRadiansPerSecond / swerve.getMaxAngularVelocity(), 0.005)
        );
    }

    public ChassisSpeeds normalizeChassisSpeeds(ChassisSpeeds autoSpeeds) {
        ChassisSpeeds normalizedSpeeds = new ChassisSpeeds(
            MathUtil.applyDeadband((normalizeSpeed(autoSpeeds.vyMetersPerSecond)), 0.01),
            -MathUtil.applyDeadband((normalizeSpeed(autoSpeeds.vxMetersPerSecond)), 0.01),
            MathUtil.applyDeadband(autoSpeeds.omegaRadiansPerSecond / swerve.getMaxAngularVelocity(), 0.005));
        Logger.recordOutput("Subsystems/Swerve/NormalizedSpeeds", normalizedSpeeds);
        return normalizedSpeeds;
    }

    public ChassisSpeeds getAutoSpeeds(Pose2d position) {
        swerve.setDesiredPose(position);
        return AutoConstants.TELE_HDC.calculate(swerve.getPose(), position, 0, position.getRotation());
    }

    public ChassisSpeeds getAutoRotationalSpeeds(Rotation2d rotation) {
        Pose2d currentPose = swerve.getPose();
        Pose2d desiredPose = new Pose2d(currentPose.getX(), currentPose.getY(), rotation);
        return getAutoSpeeds(desiredPose);
    }

    public ChassisSpeeds getProfiledAutoSpeeds(Pose2d position) {
        swerve.setDesiredPose(position);
        if (xSetpoint.position == 0 && ySetpoint.position == 0 && targetX.position == 0 && targetY.position == 0) {
            Pose2d currentPose = swerve.getPose();
            xSetpoint.position = currentPose.getX();
            ySetpoint.position = currentPose.getY();
            targetX.position = position.getX();
            targetY.position = position.getY();
        }
        position = getProfiledPosition(position);
        Logger.recordOutput("Subsystems/Swerve/StepPosition", position);
        return AutoConstants.TELE_HDC.calculate(swerve.getPose(), position, 0, position.getRotation());
    }

    public ChassisSpeeds getProfiledAutoRotationalSpeeds(Rotation2d rotation) {
        Pose2d desiredPose = new Pose2d(swerve.getPose().getX(), swerve.getPose().getY(), getProfiledRotation(rotation));
        return getProfiledAutoSpeeds(desiredPose);
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
    
    public Pose2d getProfiledPosition(Pose2d targetPose) {
        // use the profiles on the parameters
        xSetpoint = translationProfile.calculate(0.02, xSetpoint, targetX);
        ySetpoint = translationProfile.calculate(0.02, ySetpoint, targetY);
        // use the calculated states and get the positions to create a pose2d
        return new Pose2d(xSetpoint.position, ySetpoint.position, targetPose.getRotation());
    }

    public Rotation2d getProfiledRotation(Rotation2d rotation) {
        double targetTheta = rotation.getRadians();
        double posTheta = swerve.getPose().getRotation().getRadians();
        Rotation2d profiledTheta = new Rotation2d(rotationProfile.calculate(0.02, new TrapezoidProfile.State(posTheta, 0), new TrapezoidProfile.State(targetTheta, 0)).position);
        return profiledTheta;
    }

    public ChassisSpeeds getIntakeRotationalAutoSpeeds() {
        Rotation2d intakeRotation = PoseCalculations.getClosestCoralStation(swerve.getPose()).getRotation();
        return getAutoRotationalSpeeds(intakeRotation);
    }

    public ChassisSpeeds getReefRotationalAutoSpeeds() {
        Pose2d reefCenter = FieldConstants.GET_REEF_POSITION();
        Pose2d relativePose = swerve.getPose().relativeTo(reefCenter);
        Rotation2d desiredRotation;
        if (relativePose.getTranslation().getNorm() > 2d) {
            desiredRotation = new Rotation2d(relativePose.getX(), relativePose.getY());
            if (Robot.isRedAlliance()) {
                desiredRotation = desiredRotation.plus(Rotation2d.fromRadians(Math.PI));
            }
        } else {
            desiredRotation = PoseCalculations.getClosestReefSide(swerve.getPose()).getRotation();
        }
        return getAutoRotationalSpeeds(desiredRotation); // make profiled rotation2d
    }

    public ChassisSpeeds getIntakeAutoSpeeds() {
        Pose2d intakeStation = PoseCalculations.getClosestCoralStation(swerve.getPose());
        ChassisSpeeds autoSpeeds = getProfiledAutoSpeeds(intakeStation);
        return autoSpeeds;
    }
    
    public ChassisSpeeds getCageAutoSpeeds() {
        Pose2d cagePose;
        if (alignmentIndex == -1) {
            // First alignment, use nearest pose and set alignmentIndex
            int index = PoseCalculations.nearestIndex(swerve.getPose(), FieldConstants.GET_CAGE_POSITIONS());
            alignmentIndex = index;
            cagePose = FieldConstants.GET_CAGE_POSITIONS().get(index);
        } else {
            // Post-alignment, use current alignmentIndex
            cagePose = FieldConstants.GET_CAGE_POSITIONS().get(alignmentIndex);
        }
        Pose2d desiredPose = new Pose2d(
            swerve.getPose().getX(), 
            Robot.isRedAlliance() 
                ? cagePose.getY() - ClimbConstants.Y_CHASSIS_OFFSET 
                : cagePose.getY() + ClimbConstants.Y_CHASSIS_OFFSET, 
            cagePose.getRotation()
        );
        return getProfiledAutoSpeeds(desiredPose);
    }

    public ChassisSpeeds getNetAutoSpeeds() {
        boolean isRedAlliance = Robot.isRedAlliance();
        double x = isRedAlliance 
            ? FieldConstants.FIELD_MAX_LENGTH / 2 + AlgaeClawConstants.NET_X_CHASSIS_OFFSET
            : FieldConstants.FIELD_MAX_LENGTH / 2 - AlgaeClawConstants.NET_X_CHASSIS_OFFSET;
        double y = isRedAlliance
            ? MathUtil.clamp(swerve.getPose().getY(), 0, FieldConstants.FIELD_MAX_HEIGHT / 2)
            : MathUtil.clamp(swerve.getPose().getY(), FieldConstants.FIELD_MAX_HEIGHT / 2, FieldConstants.FIELD_MAX_HEIGHT);
        double theta = Robot.isRedAlliance() ? 0 : Math.PI;
        Pose2d desiredPose = new Pose2d(
            x,
            y,
            Rotation2d.fromRadians(theta)
        );
        return getAutoSpeeds(desiredPose);
    }

    // TODO: Make this less 🤮 because MY LORD
    public ChassisSpeeds getReefAxisSpeeds() {
        Pose2d currentPose = swerve.getPose();
        ReefSide reefSide = PoseCalculations.getClosestReefSide(currentPose);
        Pose2d node;
        if (alignmentIndex == -1) {
            // First alignment, find closest reef node
            node = reefSide.getCenter();
        } else {
            // Post-alignment, use current alignmentIndex
            node = alignmentIndex == 0 ? reefSide.getLeft() : reefSide.getRight();
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

        // Run motion profile to create desired pose


        Pose2d desiredPose = new Pose2d(x, y, node.getRotation());
        return getAutoSpeeds(desiredPose);  // put the pose generated by the trapazoidal profile in here (calculated with the target pose and setpoint)
    }

    public ChassisSpeeds getReefAlignmentSpeeds() {
        Pose2d currentPose = swerve.getPose();
        ReefSide reefSide = PoseCalculations.getClosestReefSide(currentPose);
        Pose2d node;
        if (alignmentIndex == -1) {
            // First alignment, go to center
            node = reefSide.getCenter();
        } else {
            // Post-alignment, use current alignmentIndex
            node = alignmentIndex == 0 ? reefSide.getLeft() : reefSide.getRight();
        }
        Logger.recordOutput("Subsystems/Swerve/TargetNode", node);
        // Get desired position from face angle
        double distance = DriveConstants.FULL_ROBOT_LENGTH_METERS / 2;
        double x = node.getX() + distance * node.getRotation().getCos();
        double y = node.getY() + distance * node.getRotation().getSin();
        Pose2d desiredPose = new Pose2d(x, y, node.getRotation());

        ChassisSpeeds autoSpeeds = getProfiledAutoSpeeds(desiredPose);
        Logger.recordOutput("Subsystems/Swerve/AutoSpeeds", autoSpeeds);
        return autoSpeeds;
    }

    public void updateIndex(int increment) {
        resetHDC();
        if (alignmentIndex == -1) {
            alignmentIndex = 0;
        }
        alignmentIndex += increment;
        // Clamp index based on number of alignment poses in the current mode
        alignmentIndex = 
            switch (alignmentMode) {
                case CAGE -> MathUtil.clamp(alignmentIndex, 0, FieldConstants.GET_CAGE_POSITIONS().size() - 1);
                case REEF -> MathUtil.clamp(alignmentIndex, 0, 1);
                default -> alignmentIndex = -1;
            };
    }

    public Command updateIndexCommand(int increment) {
        return Commands.runOnce(() -> updateIndex(increment));
    }

    public Command autoAlignmentCommand(AlignmentMode mode, Supplier<ChassisSpeeds> autoSpeeds, Supplier<ChassisSpeeds> controllerSpeeds, boolean resetAlignment) {
        return 
            Commands.sequence(
                Commands.runOnce(() -> this.alignmentMode = mode),
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
                swerve.setDesiredPose(Pose2d.kZero);
                if (resetAlignment) {
                    this.alignmentMode = AlignmentMode.NONE;
                    this.alignmentIndex = -1;
                }
            });
    }

    public Command autoAlignmentCommand(AlignmentMode mode, Supplier<ChassisSpeeds> autoSpeeds, boolean resetAlignment) {
        return 
            Commands.parallel(
                Commands.runOnce(() -> this.alignmentMode = mode),
                swerve.getDriveCommand(
                    () -> normalizeChassisSpeeds(autoSpeeds.get()),
                    () -> false
                )
            ).finallyDo(() -> {
                resetHDC();
                swerve.setDesiredPose(Pose2d.kZero);
                if (resetAlignment) {
                    this.alignmentMode = AlignmentMode.NONE;
                    this.alignmentIndex = -1;
                }
            });
    }

    public Command intakeRotationalAlignmentCommand(DoubleSupplier driverX, DoubleSupplier driverY) {
        return autoAlignmentCommand(
            AlignmentMode.INTAKE_SOFT, 
            this::getIntakeRotationalAutoSpeeds, 
            () -> getControllerSpeeds(driverX.getAsDouble(), driverY.getAsDouble()),
            true
        );
    }

    public Command reefRotationalAlignmentCommand(DoubleSupplier driverX, DoubleSupplier driverY) {
        return autoAlignmentCommand(
            AlignmentMode.REEF_SOFT,
            this::getReefRotationalAutoSpeeds,
            () -> getControllerSpeeds(driverX.getAsDouble(), driverY.getAsDouble()),
            true
        );
    }

    public Command cageAlignmentCommand(DoubleSupplier driverY) {
        return 
            autoAlignmentCommand(
                AlignmentMode.CAGE, 
                this::getCageAutoSpeeds, 
                () -> getControllerSpeeds(0, driverY.getAsDouble() * AutoConstants.CAGE_ALIGNMENT_MULTIPLIER),
                true
            );
    }

    public Command netAlignmentCommand(DoubleSupplier driverX) {
        return 
            autoAlignmentCommand(
                AlignmentMode.NET, 
                this::getNetAutoSpeeds, 
                () -> getControllerSpeeds((driverX.getAsDouble() * AutoConstants.NET_ALIGNMENT_MULTIPLIER), 0),
                true
            );
    }

    public Command reefAxisAlignmentCommand() {
        return 
            autoAlignmentCommand(
                AlignmentMode.REEF, 
                this::getReefAxisSpeeds,
                false
            );
    }

    public Command reefAlignmentCommand() {
        return
            autoAlignmentCommand(
                AlignmentMode.REEF, 
                this::getReefAlignmentSpeeds,
                true
            );
    }

    public Command intakeAlignmentCommand() {
        return
            autoAlignmentCommand(
                AlignmentMode.INTAKE, 
                this::getIntakeAutoSpeeds,
                true
            );
    }

    public Command reefFullAlignmentCommand() {
        return reefAxisAlignmentCommand().until(swerve::atHDCPose).andThen(reefAlignmentCommand()).finallyDo(() -> {
            resetHDC();
            swerve.setDesiredPose(Pose2d.kZero);
            this.alignmentMode = AlignmentMode.NONE;
            this.alignmentIndex = -1;
        });
    }

    public AlignmentMode getAlignmentMode() {
        return alignmentMode;
    }

}
