package frc.robot.util.auto;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.util.custom.ReefSide;

public class Alignment {

    private final Swerve swerve;
    @AutoLogOutput (key = "Subsystems/Swerve/AlignmentMode")
    private AlignmentMode alignmentMode = AlignmentMode.NONE;
    @AutoLogOutput (key = "Subsystems/Swerve/AlignmentIndex")
    private int alignmentIndex = -1;

    public Alignment(Swerve swerve) {
        this.swerve = swerve;
    }

    public enum AlignmentMode {
        INTAKE,
        INTAKE_SOFT,
        CAGE,
        REEF,
        REEF_SOFT,
        NET,
        PROCESSOR,
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
        return getAutoRotationalSpeeds(desiredRotation);
    }

    public ChassisSpeeds getIntakeAutoSpeeds() {
        Pose2d intakeStation = PoseCalculations.getClosestCoralStation(swerve.getPose());
        ChassisSpeeds autoSpeeds = getAutoSpeeds(intakeStation);
        double maxSpeed = AutoConstants.INTAKE_ALIGNMENT_MAX_SPEED;
        autoSpeeds.vxMetersPerSecond = MathUtil.clamp(autoSpeeds.vxMetersPerSecond, -maxSpeed, maxSpeed);
        autoSpeeds.vyMetersPerSecond = MathUtil.clamp(autoSpeeds.vyMetersPerSecond, -maxSpeed, maxSpeed);
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
        return getAutoSpeeds(desiredPose);
    }

    public ChassisSpeeds getNetAutoSpeeds() {
        boolean isRedAlliance = Robot.isRedAlliance();
        double x = isRedAlliance 
            ? FieldConstants.FIELD_MAX_LENGTH / 2 + AlgaeClawConstants.NET_X_CHASSIS_OFFSET
            : FieldConstants.FIELD_MAX_LENGTH / 2 - AlgaeClawConstants.NET_X_CHASSIS_OFFSET;
        double y = isRedAlliance
            ? MathUtil.clamp(swerve.getPose().getY(), 0, FieldConstants.FIELD_MAX_HEIGHT / 2)
            : MathUtil.clamp(swerve.getPose().getY(), FieldConstants.FIELD_MAX_HEIGHT / 2, FieldConstants.FIELD_MAX_HEIGHT);
        double theta = Robot.isRedAlliance() ? Math.PI : 0;
        Pose2d desiredPose = new Pose2d(
            x,
            y,
            Rotation2d.fromRadians(theta)
        );
        return getAutoSpeeds(desiredPose);
    }

    public ChassisSpeeds getProcessorAutoSpeeds() {
        Pose2d processor = PoseCalculations.getClosestProcessor(swerve.getPose());
        
        Pose2d desiredPose = new Pose2d(
            processor.getX() + AlgaeClawConstants.PROCESSOR_X_CHASSIS_OFFSET,
            MathUtil.clamp(swerve.getPose().getY(), 0, FieldConstants.FIELD_MAX_HEIGHT),
            processor.getRotation()
        );
        // double maxSpeed = AutoConstants.PROCESSOR_ALIGNMENT_MAX_SPEED;
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
        Pose2d desiredPose = new Pose2d(x, y, node.getRotation());
        return getAutoSpeeds(desiredPose);
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
        ChassisSpeeds autoSpeeds = getAutoSpeeds(desiredPose);
        double maxVelocity = AutoConstants.REEF_ALIGNMENT_MAX_SPEED;
        autoSpeeds.vxMetersPerSecond = MathUtil.clamp(autoSpeeds.vxMetersPerSecond, -maxVelocity, maxVelocity);
        autoSpeeds.vyMetersPerSecond = MathUtil.clamp(autoSpeeds.vyMetersPerSecond, -maxVelocity, maxVelocity);
        Logger.recordOutput("Subsystems/Swerve/AutoSpeeds", autoSpeeds);
        return autoSpeeds;
    }

    public void updateIndex(int increment) {
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


    public Command processorAlignmentCommand(DoubleSupplier driverX) {
        return
            autoAlignmentCommand(
                AlignmentMode.PROCESSOR, 
                this::getProcessorAutoSpeeds, 
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

    public Command netAlignmentCommand(DoubleSupplier driverX) {
        return
            autoAlignmentCommand(
                AlignmentMode.NET, 
                this::getNetAutoSpeeds, 
                () -> getControllerSpeeds((driverX.getAsDouble() * AutoConstants.NET_ALIGNMENT_MULTIPLIER), 0),
                true
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
