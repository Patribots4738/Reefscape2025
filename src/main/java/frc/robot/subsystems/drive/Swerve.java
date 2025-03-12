// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.DriveHDC;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleIOKraken;
import frc.robot.subsystems.drive.module.Module;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.MK4cSwerveModuleConstants;

public class Swerve extends SubsystemBase {

    public static double twistScalar = 4;
    private Rotation2d gyroRotation2d = new Rotation2d();

    private final Module frontLeft, frontRight, rearLeft, rearRight;
    // The gyro sensor
    private final Gyro gyro;

    private final Module[] swerveModules;

    private final SwerveDrivePoseEstimator poseEstimator;

    private RobotConfig config;

    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    // private final LoggedTunableNumber driveMultiplier = new LoggedTunableNumber("Swerve/DriveMultiplier", 1.0);
    // private final LoggedTunableNumber driveMaxLinearVelocity = new LoggedTunableNumber("Swerve/DriveLinearVelocity", DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    // private final LoggedTunableNumber driveMaxAngularVelocity = new LoggedTunableNumber("Swerve/DriveAngularVelocity", DriveConstants.MAX_ANGULAR_SPEED_RADS_PER_SECOND);
    // private final LoggedTunableNumber turnMaxVelocity = new LoggedTunableNumber("Swerve/MaxTurnVelocity", MK4cSwerveModuleConstants.MAX_TURNING_MOTOR_VELOCITY_RADIANS_PER_SEC);

    private ChassisSpeeds speeds = new ChassisSpeeds();
    private SwerveModuleState[] currentStates = new SwerveModuleState[4];
    private SwerveModulePosition[] currentPositions = new SwerveModulePosition[4];

    private double[] zeroFeedforwards = new double[] { 0, 0, 0, 0 };

    /**
     * Creates a new DriveSubsystem.
     */
    public Swerve() {

        frontLeft = new Module(
            new ModuleIOKraken(
                MK4cSwerveModuleConstants.FRONT_LEFT_DRIVING_CAN_ID,
                MK4cSwerveModuleConstants.FRONT_LEFT_TURNING_CAN_ID,
                MK4cSwerveModuleConstants.FRONT_LEFT_CANCODER_CAN_ID,
                MK4cSwerveModuleConstants.FRONT_LEFT_TURN_ENCODER_OFFSET),
            DriveConstants.FRONT_LEFT_INDEX,
            DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

        frontRight = new Module(
            new ModuleIOKraken(
                MK4cSwerveModuleConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                MK4cSwerveModuleConstants.FRONT_RIGHT_TURNING_CAN_ID,
                MK4cSwerveModuleConstants.FRONT_RIGHT_CANCODER_CAN_ID,
                MK4cSwerveModuleConstants.FRONT_RIGHT_TURN_ENCODER_OFFSET),
            DriveConstants.FRONT_RIGHT_INDEX,
            DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

        rearLeft = new Module(
            new ModuleIOKraken(
                MK4cSwerveModuleConstants.REAR_LEFT_DRIVING_CAN_ID,
                MK4cSwerveModuleConstants.REAR_LEFT_TURNING_CAN_ID,
                MK4cSwerveModuleConstants.REAR_LEFT_CANCODER_CAN_ID,
                MK4cSwerveModuleConstants.REAR_LEFT_TURN_ENCODER_OFFSET),
            DriveConstants.REAR_LEFT_INDEX,
            DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

        rearRight = new Module(
            new ModuleIOKraken(
                MK4cSwerveModuleConstants.REAR_RIGHT_DRIVING_CAN_ID,
                MK4cSwerveModuleConstants.REAR_RIGHT_TURNING_CAN_ID,
                MK4cSwerveModuleConstants.REAR_RIGHT_CANCODER_CAN_ID,
                MK4cSwerveModuleConstants.REAR_RIGHT_TURN_ENCODER_OFFSET),
            DriveConstants.REAR_RIGHT_INDEX,
            DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);     

        swerveModules = new Module[] {
            frontLeft,
            frontRight,
            rearLeft,
            rearRight
        };
            
        gyro = new Gyro(new GyroIOPigeon2(DriveConstants.GYRO_CAN_ID));

        resetEncoders();
        setBrakeMode();

        updateCurrentModuleStates();

        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            gyro.getYawRotation2D(),
            currentPositions,
            Pose2d.kZero,
            // State measurements
            /*
             * See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html#process-and-measurement-noise-covariance-matrices
             * for how to select the standard deviations.
             */
            // standard deviations
            // X, Y, theta
            VecBuilder.fill(
                0.003, // 6328 uses 0.003 m here
                0.003, // 6328 uses 0.003 m here
                0.0002 // 6328 uses 0.0002 rads here
            ),
            // Vision measurement
            // standard deviations
            // X, Y, theta
            VecBuilder.fill(
                0.192,
                0.192,
                Units.degreesToRadians(15)
            )
        );

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            config = new RobotConfig(
                0, 
                0, 
                new ModuleConfig(
                    0, 
                    0, 
                    0, 
                    DCMotor.getKrakenX60Foc(1), 
                    0, 
                    1
                ), 
                0
            );
            e.printStackTrace();
        }

        setpointGenerator = new SwerveSetpointGenerator(
            config, 
            MK4cSwerveModuleConstants.MAX_TURNING_MOTOR_VELOCITY_RADIANS_PER_SEC
        );

        previousSetpoint = new SwerveSetpoint(
            getRobotRelativeVelocity(), 
            currentStates, 
            DriveFeedforwards.zeros(config.numModules)
        );

        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        AutoBuilder.configure(
            this::getPose,
            this::resetOdometryAuto,
            this::getRobotRelativeVelocity,
            (speeds, feedforwards) -> driveWithSetpoints(speeds),
            AutoConstants.AUTO_HDC,
            config,
            Robot::isRedAlliance,
            this
        );
    }

    @Override
    public void periodic() {

        for (Module swerveMod : swerveModules) {
            swerveMod.updateInputs();
        }

        gyro.updateInputs();
        
        gyroRotation2d = gyro.getYawRotation2D();

        updateCurrentModuleStates();

        // TalonFX position is capped at 16000 rotations, and flips when overflowed, creating extreme pose error
        if (!getModuleDrivePositionsFlipped()) {
            poseEstimator.updateWithTime(Timer.getFPGATimestamp(), gyroRotation2d, currentPositions);
        }
        
        logData();
    }

    Pose2d currentPose = Pose2d.kZero;

    public void logData() {

        currentPose = getPose();

        RobotContainer.swerveMeasuredStates = currentStates;

        speeds = DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(RobotContainer.swerveMeasuredStates);

        if (FieldConstants.IS_SIMULATION) {
            resetOdometry(
                currentPose.exp(
                    new Twist2d(
                        0, 0,
                        speeds.omegaRadiansPerSecond * .02)
                )
            );
        }

        RobotContainer.field2d.setRobotPose(currentPose);

        if ((Double.isNaN(currentPose.getX())
            || Double.isNaN(currentPose.getY())
            || Double.isNaN(currentPose.getRotation().getDegrees())))
        {
            // Something in our pose was NaN...
            resetOdometry(RobotContainer.robotPose2d);
            resetEncoders();
        } else {
            RobotContainer.robotPose2d = currentPose;
        }

        // double pitch = gyro.getPitch();
        // double roll = gyro.getRoll();

        // Rotation3d rotation3d = FieldConstants.IS_SIMULATION 
        //     ?  new Rotation3d(
        //         Units.degreesToRadians(0), 
        //         Units.degreesToRadians(0), 
        //         currentPose.getRotation().getRadians())
        //     :  new Rotation3d(
        //         Units.degreesToRadians(roll), 
        //         Units.degreesToRadians(pitch), 
        //         currentPose.getRotation().getRadians());

        // RobotContainer.robotPose3d = new Pose3d(
        //         new Translation3d(
        //                 currentPose.getX(),
        //                 currentPose.getY(),
        //                 Math.hypot(
        //                     Rotation2d.fromDegrees(roll).getSin()
        //                             * DriveConstants.ROBOT_LENGTH_METERS / 2.0,
        //                     Rotation2d.fromDegrees(pitch).getSin()
        //                             * DriveConstants.ROBOT_LENGTH_METERS / 2.0)),
        //        rotation3d);

        Logger.recordOutput("Subsystems/Swerve/ChassisSpeeds", speeds);

    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getGyroRotation2d() {
        return this.gyroRotation2d;
    }

    public boolean getModuleDrivePositionsFlipped() {
        for (Module module : swerveModules) {
            if (module.getDrivePositionFlipped()) {
                return true;
            }
        }
        return false;
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public void drive(ChassisSpeeds robotRelativeSpeeds) {
        setModuleStates(DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.discretize(robotRelativeSpeeds, (Timer.getFPGATimestamp() - Robot.previousTimestamp)))
        );
    }

    public void driveWithSetpoints(ChassisSpeeds robotRelativeSpeeds) {
        previousSetpoint = setpointGenerator.generateSetpoint(
            previousSetpoint,
            robotRelativeSpeeds,
            Timer.getFPGATimestamp() - Robot.previousTimestamp
        );
        setModuleStates(previousSetpoint.moduleStates());
    }

    // Drive method used in teleop
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        ChassisSpeeds robotRelativeSpeeds;

        if (fieldRelative) {
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getPose().getRotation());
        } else {
            robotRelativeSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }

        drive(robotRelativeSpeeds);
    }

    public void stopDriving() {
        setWheelsX();
    }

    public void runDriveCharacterization(double input) {
        frontLeft.runDriveCharacterization(input, Units.degreesToRadians(135 + 270));
        frontRight.runDriveCharacterization(input, Units.degreesToRadians(45));
        rearLeft.runDriveCharacterization(input, Units.degreesToRadians(-135 + 180));
        rearRight.runDriveCharacterization(input, Units.degreesToRadians(-45 + 90));
    }

    public Command driveCharacterization() {
        return run(() -> runDriveCharacterization(0));
    }

    public void runTurnCharacterization(double input) {
        for (Module module : swerveModules) {
            module.runTurnCharacterization(input);
        }
    }

    public double getDriveCharacterizationVelocity() {
        double average = 0.0;
        for (Module module : swerveModules) {
            average += module.getDriveCharacterizationVelocity();
        }
        return average / 4.0;
    }

    public double getTurnCharacterizationVelocity() {
        double average = 0.0;
        for (Module module : swerveModules) {
            average += module.getTurnCharacterizationVelocity();
        }
        return average / 4.0;
    }

    public double getMaxLinearVelocity() {
        return DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    }

    public double getMaxAngularVelocity() {
        return DriveConstants.MAX_ANGULAR_SPEED_RADS_PER_SECOND;
    }

    public double getDriveMultiplier() {
        return 1.0;
    }
    
    @AutoLogOutput (key = "Subsystems/Swerve/DesiredHDCPose")
    Pose2d desiredHDCPose = Pose2d.kZero;
    public void setDesiredPose(Pose2d pose) {
        desiredHDCPose = pose;
    }

    public ChassisSpeeds getRobotRelativeVelocity() {
        return speeds;
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setWheelsX() {
        setModuleStates(DriveConstants.X_WHEEL_STATES);
    }


    public Command getSetWheelsX() {
        return run(this::setWheelsX);
    }   

    public void setWheelsO() {
        setModuleStates(DriveConstants.O_WHEEL_STATES);
    }

    public Command setWheelsOCommand() {
        return runOnce(this::setWheelsO);
    }

    public Command getSetWheelsO() {
        return run(this::setWheelsO);
    }  

    public void setWheelsZero() {
        for (Module mod : swerveModules) {
            mod.setTurnZero();
        }
    }

    public Command setWheelsZeroCommand() {
        return runOnce(this::setWheelsZero);
    }

    public Command getSetWheelsZero() {
        return run(this::setWheelsZero);
    }

    public void setTurnVelocity(double velocity) {
        for (Module mod : swerveModules) {
            mod.setTurnVelocity(velocity);
        }
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, double[] feedforwards) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            getMaxLinearVelocity()
        );
        frontLeft.setDesiredState(desiredStates[0], feedforwards[0]);
        frontRight.setDesiredState(desiredStates[1], feedforwards[1]);
        rearLeft.setDesiredState(desiredStates[2], feedforwards[2]);
        rearRight.setDesiredState(desiredStates[3], feedforwards[3]);

        RobotContainer.swerveDesiredStates = desiredStates;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        setModuleStates(desiredStates, zeroFeedforwards);
    }

    public void resetOdometry(Pose2d pose) {
        if (Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getRotation().getRadians())) {
            return;
        }

        poseEstimator.resetPosition(
            gyroRotation2d,
            currentPositions,
            pose);
    }

    // (The cameras do this for us, but we don't have cameras in sim)
    private void resetOdometryAuto(Pose2d pose) {
        if (FieldConstants.IS_SIMULATION) {
            resetOdometry(pose);
        }
    }

    public Command resetOdometryCommand(Supplier<Pose2d> pose) {
        return runOnce(() -> resetOdometry(pose.get())).ignoringDisable(true);
    }

    public Command resetPositionCommand(Supplier<Translation2d> pose) {
        return runOnce(() -> resetOdometry(new Pose2d(pose.get(), getPose().getRotation()))).ignoringDisable(true);
    }

    public void updateCurrentModuleStates() {
        for (int i = 0; i < currentStates.length; i++) {
            currentStates[i] = swerveModules[i].getState();
            currentPositions[i] = swerveModules[i].getPosition();
        }
    }
    
    public void resetEncoders() {
        for (Module swerveMod : swerveModules) {
            swerveMod.resetDriveEncoder();
        }
    }

    public double[] getWheelRadiusCharacterizationPosition() {
        return Arrays.stream(swerveModules).mapToDouble(module -> module.getDrivePositionRadians()).toArray();
    }

    /**
     * Sets the brake mode for the drive motors.
     * This is useful for when the robot is enabled
     * So we can stop the robot quickly
     * (This is the default mode)
     */
    public void setBrakeMode() {
        for (Module swerveMod : swerveModules) {
            swerveMod.setBrakeMode(true);
        }
    }

    /**
     * Sets the coast mode for the drive motors.
     * This is useful for when the robot is disabled
     * So we can freely move the robot around
     */
    public void setCoastMode() {
        for (Module swerveMod : swerveModules) {
            swerveMod.setBrakeMode(false);
        }
    }

    public Command getDriveCommand(Supplier<ChassisSpeeds> speeds, BooleanSupplier fieldRelative) {
        return new Drive(this, speeds, fieldRelative, () -> false);
    }
    
    public DriveHDC getDriveHDCCommand(Supplier<ChassisSpeeds> speeds, BooleanSupplier fieldRelative) {
        return new DriveHDC(this, speeds, fieldRelative, () -> false);
    }

    public boolean atPose(Pose2d position) {
        // More lenient on x axis, less lenient on y axis and rotation
        Pose2d currentPose = getPose();
        double angleDiff = currentPose.getRotation().minus(position.getRotation()).getRadians();
		double distance = currentPose.relativeTo(position).getTranslation().getNorm();
        return 
            MathUtil.isNear(0, distance, AutoConstants.HDC_POSITION_TOLERANCE_METERS)
            && MathUtil.isNear(0, angleDiff, AutoConstants.HDC_ROTATION_TOLERANCE_RADIANS);
    }

    public boolean atHDCPose() {
        
        return atPose(desiredHDCPose);
    }

    public boolean atHDCAngle() {
        return MathUtil.isNear(
            desiredHDCPose.getRotation().getRadians(), 
            getPose().getRotation().getRadians(),
            AutoConstants.HDC_ROTATION_TOLERANCE_RADIANS
        );
    }
}