package frc.robot.util.auto;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

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
import frc.robot.util.custom.ReefSide;

public class Alignment {

    private final Swerve swerve;
    @AutoLogOutput (key = "Subsystems/Swerve/AlignmentMode")
    private AlignmentMode alignmentMode = AlignmentMode.NONE;
    @AutoLogOutput (key = "Subsystems/Swerve/AlignmentIndex")
    private int alignmentIndex = -1;

    private TrapezoidProfile xyProfile = new TrapezoidProfile(new Constraints(AutoConstants.HDC_XY_VELOCITY, AutoConstants.HDC_XY_ACCELERATION));

    private State xySetpoint = new State(0, 0);
    private State targetXY = new State(0, 0);

    private Pose2d profileStartingPose = Pose2d.kZero;

    // private LoggedTunableNumber translationVelocity = new LoggedTunableNumber("Alignment/TranslationVelocity", AutoConstants.HDC_XY_VELOCITY);
    // private LoggedTunableNumber translationAcceleration = new LoggedTunableNumber("Alignment/TranslationAcceleration", AutoConstants.HDC_XY_ACCELERATION);

    public Alignment(Swerve swerve) {
        this.swerve = swerve;

        // translationVelocity.onChanged().or(translationAcceleration.onChanged())
        //     .onTrue(Commands.runOnce(() -> {
        //         xyProfile = new TrapezoidProfile(new Constraints(translationVelocity.get(), translationAcceleration.get()));
        //     }).ignoringDisable(true));
    }

    public enum AlignmentMode {
        INTAKE,
        INTAKE_SOFT,
        CAGE,
        REEF,
        REEF_SOFT,
        REEF_PREP,
        NET,
        PROCESSOR,
        NONE
    }

    public void resetHDC() {
        // Reset error and integral terms of HDC's PID controller
        AutoConstants.TELE_HDC.getThetaController().reset(swerve.getPose().getRotation().getRadians(), swerve.getRobotRelativeVelocity().omegaRadiansPerSecond);
        AutoConstants.TELE_HDC.getXController().reset();
        AutoConstants.TELE_HDC.getYController().reset();
    }

    public void resetProfile() {
        xySetpoint.position = 0;
        targetXY.position = 0;
        profileStartingPose = Pose2d.kZero;
    }
    
    public Command resetHDCCommand() {
        return Commands.runOnce(() -> resetHDC());
    }

    public Command resetProfileCommand() {
        return Commands.runOnce(() -> resetProfile());
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
            MathUtil.applyDeadband((normalizeTwoSpeeds(controllerSpeeds.vyMetersPerSecond, autoSpeeds.vyMetersPerSecond)), 0.005),
            -MathUtil.applyDeadband((normalizeTwoSpeeds(controllerSpeeds.vxMetersPerSecond, autoSpeeds.vxMetersPerSecond)), 0.005),
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
        Pose2d currentPose = swerve.getPose();
        if (targetXY.position == 0 || !swerve.getDesiredPose().equals(position)) {
            // First run of alignment or change of target pose, set up profile
            xySetpoint.position = 0;
            double distanceToTarget = currentPose.getTranslation().getDistance(position.getTranslation());
            targetXY.position = distanceToTarget;
            ChassisSpeeds robotSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotRelativeVelocity(), currentPose.getRotation());
            double vx = robotSpeeds.vxMetersPerSecond;
            double vy = robotSpeeds.vyMetersPerSecond;
            // Get magnitude of velocity vector
            double vMagnitude = Math.hypot(vx, vy);
            // Get direction of velocity vector with the origin at the robot
            double vTheta = Math.atan2(vy, vx);
            // Get direction of position vector with the origin at the robot
            double pTheta = Math.atan2(position.getY() - currentPose.getY(), position.getX() - currentPose.getX());
            // Get angle difference
            double theta = vTheta - pTheta;
            // Project velocity vector onto direction of position vector by calculating 
            // a new component of the velocity vector that acts in the direction of pTheta
            xySetpoint.velocity = MathUtil.clamp(vMagnitude * Math.cos(theta), 0, swerve.getMaxLinearVelocity());
            Logger.recordOutput("Subsystems/Swerve/SetpointStartVelocity", xySetpoint.velocity);
            profileStartingPose = currentPose;
        }
        swerve.setDesiredPose(position);
        position = getProfiledPosition(currentPose, position);
        Logger.recordOutput("Subsystems/Swerve/StepPosition", position);
        return AutoConstants.TELE_HDC.calculate(swerve.getPose(), position, 0, position.getRotation());
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
    
    public Pose2d getProfiledPosition(Pose2d currentPose, Pose2d targetPose) {
        xySetpoint = xyProfile.calculate(0.02, xySetpoint, targetXY);
        double magnitude = xySetpoint.position;
        double theta = Math.atan2(targetPose.getY() - profileStartingPose.getY(), targetPose.getX() - profileStartingPose.getX());
        double xOffset = magnitude * Math.cos(theta);
        double yOffset = magnitude * Math.sin(theta);
        // use the calculated states and get the positions to create a pose2d
        return new Pose2d(profileStartingPose.getX() + xOffset, profileStartingPose.getY() + yOffset, targetPose.getRotation());
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
        return getAutoSpeeds(desiredPose);
    }

    public ChassisSpeeds getNetAutoSpeeds() {
        boolean isRedAlliance = Robot.isRedAlliance();
        boolean onRedSide = PoseCalculations.isOnRedSide(swerve.getPose());
        double x = onRedSide 
            ? FieldConstants.FIELD_MAX_LENGTH / 2 + AlgaeClawConstants.NET_X_CHASSIS_OFFSET
            : FieldConstants.FIELD_MAX_LENGTH / 2 - AlgaeClawConstants.NET_X_CHASSIS_OFFSET;
        double y = isRedAlliance
            ? MathUtil.clamp(swerve.getPose().getY(), 0, FieldConstants.FIELD_MAX_HEIGHT / 2)
            : MathUtil.clamp(swerve.getPose().getY(), FieldConstants.FIELD_MAX_HEIGHT / 2, FieldConstants.FIELD_MAX_HEIGHT);
        double theta = onRedSide ? Math.PI : 0;
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
        Pose2d desiredPose = PoseCalculations.getPoseWithDistance(centerPose, distance);
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
        Pose2d desiredPose = PoseCalculations.getPoseWithDistance(node, distance);

        ChassisSpeeds autoSpeeds = getProfiledAutoSpeeds(desiredPose);
        Logger.recordOutput("Subsystems/Swerve/AutoSpeeds", autoSpeeds);
        return autoSpeeds;
    }

    public void updateIndex(int increment) {
        resetHDC();
        resetProfile();
        if (alignmentIndex == -1) {
            alignmentIndex = 0;
        }
        alignmentIndex += increment;
        // Clamp index based on number of alignment poses in the current mode
        alignmentIndex = 
            switch (alignmentMode) {
                case CAGE -> MathUtil.clamp(alignmentIndex, 0, FieldConstants.GET_CAGE_POSITIONS().size() - 1);
                case REEF, REEF_PREP -> MathUtil.clamp(alignmentIndex, 0, 1);
                default -> alignmentIndex = -1;
            };
    }

    public Command updateIndexCommand(int increment) {
        return Commands.runOnce(() -> updateIndex(increment));
    }

    public Command autoAlignmentCommand(AlignmentMode mode, Supplier<ChassisSpeeds> autoSpeeds, Supplier<ChassisSpeeds> controllerSpeeds, boolean resetAlignment) {
        return 
            Commands.sequence(
                resetHDCCommand(),
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
                resetProfile();
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
            Commands.sequence(
                resetHDCCommand(),
                Commands.runOnce(() -> this.alignmentMode = mode),
                swerve.getDriveCommand(
                    () -> normalizeChassisSpeeds(autoSpeeds.get()),
                    () -> false
                )
            ).finallyDo(() -> {
                resetHDC();
                resetProfile();
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
        return autoAlignmentCommand(
            AlignmentMode.REEF, 
            this::getReefAxisSpeeds,
            false
        );     
    }

    public Command pathfindToReefCommand() {
        return pathfindToPoseCommand(
            () -> {
                Pose2d prep = PoseCalculations.getPoseWithDistance(PoseCalculations.getClosestReefSide(swerve.getPose()).getCenter(), AutoConstants.REEF_ALIGNMENT_PREP_DISTANCE);
                Logger.recordOutput("Subsystems/Swerve/PrepPose", prep);
                return prep;
            }
        );
    }
  
    public Command pathfindToIntakeCommand() {
        return pathfindToPoseCommand(() -> PoseCalculations.getClosestCoralStation(swerve.getPose()));
    }

    public Command pathfindToPoseCommand(Supplier<Pose2d> pos) {
        return Commands.defer(() -> AutoBuilder.pathfindToPose(pos.get(), AutoConstants.prepReefConstraints, 0.0), Set.of(swerve));
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

    public Command intakeFinalAlignmentCommand() {
        return
            autoAlignmentCommand(
                AlignmentMode.INTAKE, 
                this::getIntakeAutoSpeeds, 
                true
            );
    }

    public Command intakeAlignmentCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> this.alignmentMode = AlignmentMode.INTAKE),
            pathfindToIntakeCommand(),
            // .until(() -> swerve.getPose().getTranslation().getDistance(PoseCalculations.getClosestCoralStation(swerve.getPose()).getTranslation()) < 1.5),
            intakeFinalAlignmentCommand()
        );
    }

    public Command reefFullAlignmentCommand() {
        return reefAlignmentCommand();
    }

    public AlignmentMode getAlignmentMode() {
        return alignmentMode;
    }

    public boolean shouldPathfindToReef() {
        Pose2d currentPose = swerve.getPose();
        Pose2d reefFace = PoseCalculations.getClosestReefSide(currentPose).getCenter();
        double linearDistance = currentPose.getTranslation().getDistance(reefFace.getTranslation());
        return linearDistance > (AutoConstants.REEF_ALIGNMENT_PREP_DISTANCE);
    }

}
