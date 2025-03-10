// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.custom.LoggedGainConstants;
import frc.robot.util.custom.ReefSide;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.rev.Neo;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class LoggingConstants {

        private static RobotType robotType = RobotType.DEVBOT;

        public static RobotType getRobot() {
            if (FieldConstants.IS_REAL && robotType == RobotType.SIMBOT) {
                System.out.println("Incorrect robot type selected, changing to real robot");
                robotType = RobotType.COMPBOT;
            }
            return robotType;
        }

        public static Mode getMode() {
            return switch (getRobot()) {
                case DEVBOT -> FieldConstants.IS_SIMULATION ? Mode.SIM : Mode.REAL;
                case COMPBOT -> FieldConstants.IS_SIMULATION ? Mode.REPLAY : Mode.REAL;
                case SIMBOT -> Mode.SIM;
            };
        }

        public enum Mode {
            /** Running on a real robot. */
            REAL,
            /** Running a physics simulator. */
            SIM,
            /** Replaying from a log file. */
            REPLAY
        }

        public enum RobotType {
            DEVBOT,
            COMPBOT,
            SIMBOT
        }

        public static final int CLIMB_INDEX = 0;
        public static final int ELEVATOR_FIRST_STAGE_INDEX = 1;
        public static final int ELEVATOR_SECOND_STAGE_INDEX = 2;
        public static final int WRIST_INDEX = 3;

        public static final Translation3d ROBOT_OFFSET = new Translation3d(
            0.0,
            0.0,
            0.0
        );

        public static final Translation3d CLIMB_OFFSET = new Translation3d(
            ROBOT_OFFSET.getX() + 0.056, 
            ROBOT_OFFSET.getY() + 0.241, 
            ROBOT_OFFSET.getZ() + 0.2735
        );

        public static final Translation3d WRIST_OFFSET = new Translation3d(
            ROBOT_OFFSET.getX() - 0.166, 
            ROBOT_OFFSET.getY() + 0.007, 
            ROBOT_OFFSET.getZ() + 0.7836464896
        );

        public static final Translation3d CORAL_OFFSET = new Translation3d(
            0.117,
            -0.005,
            -0.154653
        );

        public static final Translation3d ALGAE_OFFSET = new Translation3d(
            -0.325,
            0,
            -0.05
        );

    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double MAX_SPEED_METERS_PER_SECOND = 4.5;

        public static final double MAX_ANGULAR_SPEED_RADS_PER_SECOND = Units.degreesToRadians(1137.21); // radians per second

        public static final double ODOMETRY_FREQUENCY = 250.0;

        // Chassis configuration
        // Distance between centers of right and left wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(23.0);
        // Distance between front and back wheels on robot
        // Easiest measured from the center of the bore of the vortex
        public static final double WHEEL_BASE = Units.inchesToMeters(23.0);

        public static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(28.5);
        public static final double BUMPER_LENGTH_METERS = Units.inchesToMeters(3.5);
        public static final double FULL_ROBOT_LENGTH_METERS = Units.inchesToMeters(36.5865);

        // Front positive, left positive
        public static final Translation2d FRONT_LEFT_WHEEL_POSITION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d FRONT_RIGHT_WHEEL_POSITION = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
        public static final Translation2d REAR_LEFT_WHEEL_POSITION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d REAR_RIGHT_WHEEL_POSITION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

        public static final Translation2d[] WHEEL_POSITION_ARRAY = new Translation2d[] {
            FRONT_LEFT_WHEEL_POSITION,
            FRONT_RIGHT_WHEEL_POSITION,
            REAR_LEFT_WHEEL_POSITION,
            REAR_RIGHT_WHEEL_POSITION
        };

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                WHEEL_POSITION_ARRAY
        );

        public static final SwerveModuleState[] X_WHEEL_STATES = new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-135))
        };

        public static final SwerveModuleState[] O_WHEEL_STATES = new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        };

        public static final ChassisSpeeds ZEROED_SPEEDS = new ChassisSpeeds();
        public static final ChassisSpeeds MAX_SPEEDS = new ChassisSpeeds(DriveConstants.MAX_SPEED_METERS_PER_SECOND, DriveConstants.MAX_SPEED_METERS_PER_SECOND, DriveConstants.MAX_ANGULAR_SPEED_RADS_PER_SECOND);

        // Angular offsets of the modules relative to the chassis in radians
        // add 90 degrees to change the X and Y axis
        public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(180 + 90);
        public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(-90 + 90);
        public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(90 + 90);
        public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(0 + 90);

        public static final int GYRO_CAN_ID = 29;
        public static final boolean GYRO_REVERSED = true;

        public static final int FRONT_LEFT_INDEX = 0;
        public static final int FRONT_RIGHT_INDEX = 1;
        public static final int REAR_LEFT_INDEX = 2;
        public static final int REAR_RIGHT_INDEX = 3;

    }

    public static final class AutoConstants {

        //allignment trapazoidal profile constants
        public static final double HDC_XY_ACCELERATION = 2.5;
        public static final double HDC_XY_VELOCITY = 1;

        public static final double HDC_THETA_ACCELERATION =  Units.degreesToRadians(450d);
        public static final double HDC_THETA_VELOCITY =  Units.degreesToRadians(270d);

        public static final String REEF_NODES = "ABCDEFGHIJKL";

        public static final PathConstraints prepReefConstraints = new PathConstraints(
                2.350, 
                4.100, 
                Units.degreesToRadians(270), 
                Units.degreesToRadians(450)
            );

        public static final double HDC_POSITION_TOLERANCE_METERS = Units.inchesToMeters(1);
        public static final double HDC_ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(2);

        public static final double REEF_ALIGNMENT_MAX_SPEED = 1.0;
        public static final double INTAKE_ALIGNMENT_MAX_SPEED = 1.0;

        public static final double REEF_ALIGNMENT_PREP_DISTANCE = 1.0;


        public static final GainConstants AUTO_XY_GAINS = new GainConstants(
            6, 
            0.0, 
            0.09
        );

        public static final GainConstants TELE_XY_GAINS = new GainConstants(
            4.0, 
            0, 
            0.0
        );

        public static final LoggedGainConstants LOGGED_TELE_XY_GAINS = new LoggedGainConstants(AutoConstants.TELE_XY_GAINS, "TeleXY");

        public static final GainConstants AUTO_THETA_GAINS = new GainConstants(
            3.725, 
            0, 
            0
        );

        public static final GainConstants TELE_THETA_GAINS = new GainConstants(
            5.5, 
            0, 
            0
        );

        public static final LoggedGainConstants LOGGED_TELE_THETA_GAINS = new LoggedGainConstants(AutoConstants.TELE_THETA_GAINS, "TeleTheta");

        public static final PIDController XY_PID = new PIDController(
            AutoConstants.TELE_XY_GAINS.getP(),
            AutoConstants.TELE_XY_GAINS.getI(),
            AutoConstants.TELE_XY_GAINS.getD()
        );


        public static final ProfiledPIDController THETA_PID = new ProfiledPIDController(
            AutoConstants.TELE_THETA_GAINS.getP(),
            AutoConstants.TELE_THETA_GAINS.getI(),
            AutoConstants.TELE_THETA_GAINS.getD(),
            new TrapezoidProfile.Constraints(
                AutoConstants.HDC_THETA_VELOCITY,
                AutoConstants.HDC_THETA_ACCELERATION)) 
            {{
                setIZone(Units.degreesToRadians(45));
            }};

        public static HolonomicDriveController TELE_HDC = new HolonomicDriveController(
            XY_PID,
            XY_PID,
            THETA_PID
        );

        public static PPHolonomicDriveController AUTO_HDC = new PPHolonomicDriveController(
            new PIDConstants(
                AutoConstants.AUTO_XY_GAINS.getP(),
                AutoConstants.AUTO_XY_GAINS.getI(),
                AutoConstants.AUTO_XY_GAINS.getD()),
            new PIDConstants(
                AutoConstants.AUTO_THETA_GAINS.getP(),
                AutoConstants.AUTO_THETA_GAINS.getI(),
                AutoConstants.AUTO_THETA_GAINS.getD(),
                Units.degreesToRadians(45)));

        public static final String[] AUTO_NAMES = new String[] {
            "5-E4-D4-C4-B4",
            "3-J4-K4-L4-A4"
        };

        public static final double REEF_ALIGNMENT_MULTIPLIER = 0.5;
        public static final double CAGE_ALIGNMENT_MULTIPLIER = 0.5;
        public static final double NET_ALIGNMENT_MULTIPLIER = 0.5;

    }

    public static final class MAXSwerveModuleConstants {
        // https://www.revrobotics.com/rev-21-3005/
        private enum SwerveGearing {
            LOW         (12, 22),
            MEDIUM      (13, 22),
            HIGH        (14, 22),

            EXTRA_HIGH_1(14, 21),
            EXTRA_HIGH_2(14, 20),
            EXTRA_HIGH_3(15, 20),
            EXTRA_HIGH_4(16, 20),
            EXTRA_HIGH_5(16, 19);

            private final double 
                pinionTeeth, 
                spurTeeth;

            SwerveGearing(
                int pinionTeeth, 
                int spurTeeth)
            {
                this.pinionTeeth = pinionTeeth;
                this.spurTeeth = spurTeeth;
            }
            
        }

        public static final SwerveGearing CURRENT_GEARING = SwerveGearing.HIGH;

        // Driving motors CAN IDs (EVEN)
        public static final int FRONT_LEFT_DRIVING_CAN_ID = 3;
        public static final int REAR_LEFT_DRIVING_CAN_ID = 5;
        public static final int FRONT_RIGHT_DRIVING_CAN_ID = 1;
        public static final int REAR_RIGHT_DRIVING_CAN_ID = 7;

        // Turning motors CAN IDs (ODD)
        public static final int FRONT_LEFT_TURNING_CAN_ID = 4;
        public static final int REAR_LEFT_TURNING_CAN_ID = 6;
        public static final int FRONT_RIGHT_TURNING_CAN_ID = 2;
        public static final int REAR_RIGHT_TURNING_CAN_ID = 8;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean TURNING_ENCODER_INVERTED = true;

        public static final double MAX_TURNING_MOTOR_VELOCITY_RADIANS_PER_SEC = 20.0 * Math.PI;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.VORTEX_FREE_SPEED_RPM / 60;
        // **********************************************************************MAX SWERVE**********************
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(1.4642497827983136*2.0);
        // **********************************************************************MAX SWERVE**********************
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        // 45 teeth on the wheel's bevel gear, 15 teeth on the bevel pinion
        public static final double DRIVING_MOTOR_REDUCTION = (45.0 * CURRENT_GEARING.spurTeeth) / (CURRENT_GEARING.pinionTeeth * 15.0);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
                * WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION;

        public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR = (WHEEL_CIRCUMFERENCE_METERS
                / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
        public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

        public static final double DRIVING_P = 0.256;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0.255;
        public static final double DRIVING_FF = 0.20217;
        public static final double DRIVING_MIN_OUTPUT = -1;
        public static final double DRIVING_MAX_OUTPUT = 1;
        public static final GainConstants DRIVING_PID = new GainConstants(
            DRIVING_P,
            DRIVING_I,
            DRIVING_D,
            DRIVING_FF,
            DRIVING_MIN_OUTPUT,
            DRIVING_MAX_OUTPUT
        );

        public static final double TURNING_P = Robot.isSimulation() ? 0.5 : 1.5;
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 1;
        public static final double TURNING_FF = 0;
        public static final double TURNING_MIN_OUTPUT = -1;
        public static final double TURNING_MAX_OUTPUT = 1;
        public static final GainConstants TURNING_PID = new GainConstants(
            TURNING_P,
            TURNING_I,
            TURNING_D,
            TURNING_FF,
            TURNING_MIN_OUTPUT,
            TURNING_MAX_OUTPUT
        );

        public static final int NEO_CURRENT_LIMIT = 50; // amps
        public static final int VORTEX_CURRENT_LIMIT = 80; // amps
        public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps

    }

    public static final class MK4cSwerveModuleConstants {
        // Driving motors CAN IDs (EVEN)
        public static final int FRONT_LEFT_DRIVING_CAN_ID = 3;
        public static final int REAR_LEFT_DRIVING_CAN_ID = 5;
        public static final int FRONT_RIGHT_DRIVING_CAN_ID = 1;
        public static final int REAR_RIGHT_DRIVING_CAN_ID = 7;

        // Turning motors CAN IDs (ODD)
        public static final int FRONT_LEFT_TURNING_CAN_ID = 4;
        public static final int REAR_LEFT_TURNING_CAN_ID = 6;
        public static final int FRONT_RIGHT_TURNING_CAN_ID = 2;
        public static final int REAR_RIGHT_TURNING_CAN_ID = 8;

        // CANcoders CAN IDs
        public static final int FRONT_LEFT_CANCODER_CAN_ID = 22;
        public static final int REAR_LEFT_CANCODER_CAN_ID = 23;
        public static final int FRONT_RIGHT_CANCODER_CAN_ID = 21;
        public static final int REAR_RIGHT_CANCODER_CAN_ID = 24;

        private enum SwerveGearing {

            L1(7.13),
            L2(5.9),
            L3(5.36);

            private final double gearRatio;

            SwerveGearing(double gearRatio) {
                this.gearRatio = gearRatio;
            }

            
        };
        
        public static final double LINEAR_VELOCITY_DEADBAND = 0.005;
        public static final double ANGULAR_VELOCITY_DEADBAND = 0.003;
        
        public static final SwerveGearing CURRENT_GEARING = SwerveGearing.L2;

        public static final double FRONT_LEFT_TURN_ENCODER_OFFSET = 0.28271484375;
        public static final double FRONT_RIGHT_TURN_ENCODER_OFFSET = -0.283936;
        public static final double REAR_LEFT_TURN_ENCODER_OFFSET = 0.081055;
        public static final double REAR_RIGHT_TURN_ENCODER_OFFSET = 0.139404296875;

        public static final double TURNING_MOTOR_REDUCTION = 12.8;

        public static final double MAX_TURNING_MOTOR_VELOCITY_RADIANS_PER_SEC = 609.669;

        public static final boolean INVERT_TURNING_MOTOR = false;

        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = KrakenMotorConstants.KRAKENX60_FREE_SPEED_RPM_FOC / 60;
        // **********************************************************************MK4c SWERVE**********************
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(1.9121028598030858 * 2);
        // **********************************************************************MK4c SWERVE**********************
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

        public static final double DRIVE_GEAR_RATIO = CURRENT_GEARING.gearRatio;

        public static final double DRIVING_ENCODER_POSITION_FACTOR = WHEEL_CIRCUMFERENCE_METERS; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR = WHEEL_CIRCUMFERENCE_METERS; // meters per second

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI); // radians per second

        public static final double DRIVING_MOTOR_STATOR_LIMIT_AMPS = 80.0;
        public static final double DRIVING_MOTOR_SUPPLY_LIMIT_AMPS = 80.0;
        public static final double TURNING_MOTOR_STATOR_LIMIT_AMPS = 60.0;
        public static final double TURNING_MOTOR_SUPPLY_LIMIT_AMPS = 60.0;
        public static final double DRIVING_MOTOR_TORQUE_LIMIT_AMPS = 80.0;
        public static final double TURNING_MOTOR_TORQUE_LIMIT_AMPS = 60.0;

        public static final double DRIVING_P = 35d;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0;
        public static final double DRIVING_S = 2.4316460000000006;
        public static final double DRIVING_V = 0;

        public static final GainConstants DRIVING_GAINS = new GainConstants(
            DRIVING_P,
            DRIVING_I,
            DRIVING_D,
            DRIVING_S,
            DRIVING_V,
            0.0
        );

        // public static final LoggedGainConstants LOGGED_DRIVING_GAINS = new LoggedGainConstants(MK4cSwerveModuleConstants.DRIVING_GAINS, "Swerve/Drive");

        public static final double TURNING_P = FieldConstants.IS_REAL ? 1500d : 2400d;
        public static final double TURNING_I = 0;
        public static final double TURNING_D = FieldConstants.IS_REAL ? 30d : 250d;
        public static final double TURNING_S = FieldConstants.IS_REAL ? 2.800419999999998 : 1.0199330000001938;

        public static final GainConstants TURNING_GAINS = new GainConstants(
            TURNING_P,
            TURNING_I,
            TURNING_D,
            TURNING_S,
            0.0,
            0.0
        );

        // public static final LoggedGainConstants LOGGED_TURNING_GAINS = new LoggedGainConstants(MK4cSwerveModuleConstants.TURNING_GAINS, "Swerve/Turn");

    }

    public static final class CoralClawConstants {

        public static final int CAN_ID = 9;

        public static final boolean BRAKE_MOTOR = true;
        public static final double CURRENT_LIMIT = 45.0;

        public static final boolean MOTOR_INVERTED = false;

        public static final double INTAKE_PERCENT = 0.4;
        public static final double HOLD_PERCENT = 0.1;
        public static final double OUTTAKE_PERCENT = -0.75;
        public static final double L1_OUTTAKE_PERCENT = -0.1;

        public static final double PLACING_NAMED_COMMAND_TIME = 0.5;

        public static final double CURRENT_THRESHOLD_HAS_PIECE_AMPS = 5.0;

        public static final double CORAL_CLAW_CURRENT_DEADBAND = 15d;

        public static final double HAS_PIECE_INTAKE_THRESHOLD = 5d;

        public static final double HAS_PIECE_OUTTAKE_THRESHOLD_1 = 30d;
        public static final double HAS_PIECE_OUTTAKE_THRESHOLD_2 = 7d;

    }

    public static final class AlgaeClawConstants {

        public static final int CAN_ID = 14;

        public static final boolean BRAKE_MOTOR = true;
        public static final double CURRENT_LIMIT = 80.0;

        public static final boolean MOTOR_INVERTED = false;

        public static final double HOLD_PERCENT = 0.75;
        public static final double INTAKE_PERCENT = 1.0;
        public static final double OUTTAKE_PERCENT = -1.0;

        public static final double PLACING_NAMED_COMMAND_TIME = 0.5;

        public static final double NET_X_CHASSIS_OFFSET = 0.61;

    }
    public static final class ElevatorConstants {

        public static final int LEADER_CAN_ID = 10;
        public static final int FOLLOWER_CAN_ID = 11;

        public static final boolean BRAKE_MOTOR = true;
        public static final double CURRENT_LIMIT = 80.0;

        public static final boolean MOTOR_INVERTED = false;

        public static final double GEAR_RATIO = 9.0;
        public static final double MAX_DISPLACEMENT_METERS = 0.58;
        public static final double POSITION_CONVERSION_FACTOR = Units.inchesToMeters(6.01716904509);
        public static final double VELOCITY_CONVERSION_FACTOR = Units.inchesToMeters(6.01716904509);

        public static final double P = FieldConstants.IS_REAL ? 400d : 800d;
        public static final double I = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double D = FieldConstants.IS_REAL ? 30d : 200d;
        public static final double A = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double S = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double V = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double G = FieldConstants.IS_REAL ? 10d : 0d;

        public static final GainConstants GAINS = new GainConstants(
            P,
            I,
            D,
            A,
            S,
            V,
            G
        );

        public static final LoggedGainConstants LOGGED_GAINS = new LoggedGainConstants(ElevatorConstants.GAINS, "Elevator");
        
        public static final double FAST_VELOCITY = 1.6;
        public static final double FAST_ACCELERATION = 8d;
        public static final double SLOW_VELOCITY = 1.6;
        public static final double SLOW_ACCELERATION = 4d;
        public static final double JERK = 40d;

        public static final double STOW_POSITION_METERS = 0.0;
        public static final double INTAKE_POSITION_METERS = 0.0;
        public static final double L1_POSITION_METERS = 0.1;
        public static final double L2_POSITION_METERS = 0.09;
        public static final double L3_POSITION_METERS = 0.27;
        public static final double L4_POSITION_METERS = MAX_DISPLACEMENT_METERS;
        public static final double L3_POSITION_REMOVE_ALGAE = 0.13;
        public static final double L2_POSITION_REMOVE_ALGAE = 0.04;
        public static final double PROCESSOR_METERS = 0;
        public static final double NET_PREP_METERS = 0.32;
        public static final double NET_METERS = MAX_DISPLACEMENT_METERS;

        public static final double DEADBAND_METERS = 0.02;

    }


    public static final class WristConstants {

        public static final int CAN_ID = 12;
        public static final int ENCODER_DIO_PIN = 9;

        public static final double ENCODER_POSITION_OFFSET_ROTATIONS = 2.575;
        public static final boolean ENCODER_INVERTED = false;

        public static final boolean BRAKE_MOTOR = true;

        public static final boolean MOTOR_INVERTED = false;

        public static final double P = FieldConstants.IS_REAL ? 1500d : 5000d;
        public static final double I = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double D = FieldConstants.IS_REAL ? 100d : 500d;
        public static final double A = FieldConstants.IS_REAL ? 3d : 0d;
        public static final double S = FieldConstants.IS_REAL ? 1.5052 : 0d;
        public static final double V = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double G = FieldConstants.IS_REAL ? 5.6 : 0d;

        public static final GainConstants GAINS = new GainConstants(
            P,
            I,
            D,
            A,
            S,
            V,
            G
        );

        // public static final LoggedGainConstants LOGGED_GAINS = new LoggedGainConstants(WristConstants.GAINS, "Wrist");

        public static final double FAST_VELOCITY = 11d;
        public static final double FAST_ACCELERATION = FieldConstants.IS_REAL ? 90d : 15d;
        public static final double SLOW_VELOCITY = 5d;
        public static final double SLOW_ACCELERATION = FieldConstants.IS_REAL ? 10d : 15d;
        public static final double JERK = 900d;

        public static final double GEAR_RATIO = 60.0;
    
        public static final double POSITION_CONVERSION_FACTOR = 2 * Math.PI;
        public static final double VELOCITY_CONVERSION_FACTOR = 2 * Math.PI;
        public static final double ENCODER_POSITION_CONVERSION_FACTOR = 2 * Math.PI;

        public static final double CURRENT_LIMIT = 60.0;

        public static final double CG_OFFSET_ANGLE_RADIANS = 0.187;

        public static final double MIN_ANGLE_RADIANS = -1.11370457898;
        public static final double MAX_ANGLE_RADIANS = 2.75;

        public static final double CLIMB_RADIANS = 1.1;

        public static final double DEADBAND_RADIANS = 0.06;
        public static final double STOW_POSITION_RADIANS = 0.0;
        public static final double INTAKE_POSITION_RADIANS = -0.355;
        public static final double L1_POSITION_RADIANS = 1.8;
        public static final double L1_PLACE_POSITION_RADIANS = 2.75;
        public static final double L2_POSITION_RADIANS = 2.41;
        public static final double L3_POSITION_RADIANS = 2.41;
        public static final double L4_POSITION_RADIANS = 2.32;
        public static final double L2_ALGAE_REMOVAL = 0.7;
        public static final double L3_ALGAE_REMOVAL = 1.2;
        public static final double BACK_ALGAE_TOSS = 0;
        public static final double FRONT_ALGAE_TOSS = 2.0;
        public static final double PROCESSOR_RADIANS = -0.85;

        public static final double TRANSITION_RADIANS = 1.57;
        public static final double UNDER_THRESHOLD_RADIANS = 1.2;

        public static final double NET_RADIANS = 1.7;
    }


    public static final class ClimbConstants {

        public static final int CAN_ID = 13;

        public static final boolean MOTOR_INVERTED = false;

        public static final double GEAR_RATIO = 268.8;

        public static final double POSITION_CONVERSION_FACTOR = 2.0 * Math.PI;
        public static final double VELOCITY_CONVERSION_FACTOR = 2.0 * Math.PI;

        public static final double CURRENT_LIMIT = 120.0;
        public static final boolean BRAKE_MOTOR = true;

        public static final double MIN_ANGLE_RADIANS = 0.0;
        public static final double MAX_ANGLE_RADIANS = 2.5;

        public static final double P_SLOW = FieldConstants.IS_REAL ? 100d : 1000d; // Chef Cushman
        public static final double I_SLOW = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double D_SLOW = FieldConstants.IS_REAL ? 2d : 500d;
        public static final double A_SLOW = FieldConstants.IS_REAL ? 1d : 0d;
        public static final double S_SLOW = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double V_SLOW = FieldConstants.IS_REAL ? 0d : 0d; 
        public static final double G_SLOW = FieldConstants.IS_REAL ? 0d : 0d;

        public static final double P_FAST = FieldConstants.IS_REAL ? 100d : 1000d;
        public static final double I_FAST = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double D_FAST = FieldConstants.IS_REAL ? 2d : 500d;
        public static final double A_FAST = FieldConstants.IS_REAL ? 1d : 0d;
        public static final double S_FAST = FieldConstants.IS_REAL ? 0d : 0d; 
        public static final double V_FAST = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double G_FAST = FieldConstants.IS_REAL ? -50d : 0d;

        public static final double VELOCITY = Math.PI;
        public static final double ACCELERATION = 20d;
        public static final double JERK = 200d;

        public static final GainConstants SLOW_GAINS = new GainConstants(
            P_SLOW,
            I_SLOW,
            D_SLOW,
            A_SLOW,
            S_SLOW,
            V_SLOW,
            G_SLOW
        );

        public static final GainConstants FAST_GAINS = new GainConstants(
            P_FAST,
            I_FAST,
            D_FAST,
            A_FAST,
            S_FAST,
            V_FAST,
            G_FAST
        );

        public static final LoggedGainConstants LOGGED_SLOW_GAINS = new LoggedGainConstants(ClimbConstants.SLOW_GAINS, "Climb/SlowGains");
        public static final LoggedGainConstants LOGGED_FAST_GAINS = new LoggedGainConstants(ClimbConstants.FAST_GAINS, "Climb/FastGains");
        
        public static final double STOW_POSITION_RADIANS = 0.0;
        public static final double READY_POSITION_RADIANS = 2.5;
        public static final double FINAL_POSITION_RADIANS = 0.7;

        public static final double Y_CHASSIS_OFFSET = 0.0508;

        public static final double DEADBAND_RADIANS = Units.degreesToRadians(3.0);

    }

    public static final class OIConstants {

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final double DRIVER_DEADBAND = 0.08;
        public static final double OPERATOR_DEADBAND = 0.08;

        // See https://www.desmos.com/calculator/e07raajzh5
        // And
        // https://docs.google.com/spreadsheets/d/1Lytrh6q9jkz4u1gmF1Sk8kTpj8DxW-uwRE_QMnTt8Lk
        public static final double CONTROLLER_CORNER_SLOPE_1 = 1 / 0.7;
        public static final double CONTROLLER_CORNER_SLOPE_2 = 0.7;

        public static final double DRIVER_ALIGN_CANCEL_DEADBAND = 0.15;

        public enum DriverMode {
            DOUBLE,
            DEV,
            CALIBRATION
        }

        public static final DriverMode DRIVER_MODE = DriverMode.DOUBLE;

    }

    public static final class CameraConstants {

        public static final int ENABLED_THROTTLE = 0; // Process every frame
        public static final int DISABLED_THROTTLE = 30; // Process 1 of every 80 frames

        public static Pose3d LL4_POSE = new Pose3d(
            -0.28702,
            0,
            0.1911680708,
            new Rotation3d(
                0, 
                15, 
                180
            )
        );

        public static Pose3d LL3G_POSE = new Pose3d(
            -0.0968974694,
            0.28506928,
            0.5413440786,
            new Rotation3d(
                0,
                15,
                0
            )
        );

    }

    public static final class NeoMotorConstants {
        public static final double VORTEX_FREE_SPEED_RPM = 6784;

        public static ArrayList<SparkBase> motors = new ArrayList<SparkBase>();

        public static final boolean SAFE_SPARK_MODE = false;
        public static final double NEO_FREE_SPEED_RPM = 5676;

        public static final int MAX_PERIODIC_STATUS_TIME_MS = 32766;
        public static final int FAST_PERIODIC_STATUS_TIME_MS = 15;
        // This gets filled out as motors are created on the robot
        public static final HashMap<Integer, Neo> NEO_MOTOR_MAP = new HashMap<Integer, Neo>();

        public static final HashMap<Integer, String> CAN_ID_MAP = new HashMap<Integer, String>() {{
            /*  1  */ put(MAXSwerveModuleConstants.FRONT_LEFT_DRIVING_CAN_ID, "FrontRightDrive");
            /*  2  */ put(MAXSwerveModuleConstants.FRONT_RIGHT_TURNING_CAN_ID, "FrontRightTurn");
            /*  3  */ put(MAXSwerveModuleConstants.FRONT_LEFT_DRIVING_CAN_ID, "FrontLeftDrive");
            /*  4  */ put(MAXSwerveModuleConstants.FRONT_LEFT_TURNING_CAN_ID, "FrontLeftTurn");
            /*  5  */ put(MAXSwerveModuleConstants.REAR_LEFT_DRIVING_CAN_ID, "RearLeftDrive");
            /*  6  */ put(MAXSwerveModuleConstants.REAR_LEFT_TURNING_CAN_ID, "RearLeftTurn");
            /*  7  */ put(MAXSwerveModuleConstants.REAR_RIGHT_DRIVING_CAN_ID, "RearRightDrive");
            /*  8  */ put(MAXSwerveModuleConstants.REAR_RIGHT_TURNING_CAN_ID, "RearRightTurn");
        }};

        public static final HashMap<String, List<Neo>> NEO_MOTOR_GROUPS = new HashMap<String, List<Neo>>();

        public static Map<String, List<Neo>> initializeMotorGroupMap() {
            // NEO_MOTOR_GROUPS.put("Drive", new ArrayList<Neo>() {{
            //     add(NEO_MOTOR_MAP.get(MAXSwerveModuleConstants.FRONT_LEFT_DRIVING_CAN_ID));
            //     add(NEO_MOTOR_MAP.get(MAXSwerveModuleConstants.FRONT_RIGHT_DRIVING_CAN_ID));
            //     add(NEO_MOTOR_MAP.get(MAXSwerveModuleConstants.REAR_LEFT_DRIVING_CAN_ID));
            //     add(NEO_MOTOR_MAP.get(MAXSwerveModuleConstants.REAR_RIGHT_DRIVING_CAN_ID));
            // }});
            // NEO_MOTOR_GROUPS.put("Turn", new ArrayList<Neo>() {{
            //     add(NEO_MOTOR_MAP.get(MAXSwerveModuleConstants.FRONT_LEFT_TURNING_CAN_ID));
            //     add(NEO_MOTOR_MAP.get(MAXSwerveModuleConstants.FRONT_RIGHT_TURNING_CAN_ID));
            //     add(NEO_MOTOR_MAP.get(MAXSwerveModuleConstants.REAR_LEFT_TURNING_CAN_ID));
            //     add(NEO_MOTOR_MAP.get(MAXSwerveModuleConstants.REAR_RIGHT_TURNING_CAN_ID));
            // }});

            return NEO_MOTOR_GROUPS;
        }

    }

    public static final class KrakenMotorConstants {

        public static final double KRAKENX60_FREE_SPEED_RPM = 6000;
        public static final double KRAKENX60_FREE_SPEED_RPM_FOC = 5800;

        public static final double TALONFX_FAST_UPDATE_FREQ_HZ = 75;// TODO: FIND THE SWEET SPOT
        public static final double TALONFX_MID_UPDATE_FREQ_HZ = 50; // TODO: FIND THE SWEET SPOT
        public static final double TALONFX_SLOW_UPDATE_FREQ_HZ = 4; // TODO: FIND THE SWEET SPOT

        public static final HashMap<Integer, Kraken> KRAKEN_MOTOR_MAP = new HashMap<Integer, Kraken>();

    }

    public static final class CANCoderConstants {

        public static final double ENCODER_UPDATE_FREQ_HZ = 100; // TODO: FIND THE SWEET SPOT

    }

    public static final class PigeonConstants {

        public static final double PIGEON_FAST_UPDATE_FREQ_HZ = 250; // TODO: FIND THE SWEET SPOT

    }

    public static final class GeneralHardwareConstants {
        public static final boolean SAFE_HARDWARE_MODE = false;
        public static final double TIMEOUT_SECONDS = 1.0;
    }

    public static final class FieldConstants {

        public static final boolean IS_SIMULATION = Robot.isSimulation();
        public static final boolean IS_REAL = !IS_SIMULATION;

        public static final double FIELD_MAX_LENGTH = 17.55;
        // 2d height
        public static final double FIELD_MAX_HEIGHT = 8.0518;

        public static final double INTAKE_ALIGNMENT_DISTANCE_METERS = 1d;

        // All These Positions Are For The Blue Side Unless Specified Otherwise

        public static final Pose2d BLUE_PROCESSOR = new Pose2d(6.00, 0.00, Rotation2d.fromRadians(0));

        public static final List<Pose2d> PROCESSOR_POSITIONS = new ArrayList<Pose2d>() {{
            // Blue Processor
            add(BLUE_PROCESSOR);
            // Red Processor
            add(PoseCalculations.mirrorPose(BLUE_PROCESSOR));
        }};

        public static final Pose2d GET_PROCESSOR_POSITION() {
            int index = Robot.isRedAlliance() ? 1 : 0;
            return PROCESSOR_POSITIONS.get(index);
        }

        public static final Pose2d CORAL_STATION_1 = new Pose2d(1.653, 7.364, Rotation2d.fromDegrees(125));
        public static final Pose2d CORAL_STATION_2 = new Pose2d(1.653, 0.699, Rotation2d.fromDegrees(-125));

        public static final double CORAL_STATION_HEIGHT = 0.95;

        public static final List<Pose2d> CORAL_STATION_POSITIONS = new ArrayList<Pose2d>() {{

            Pose2d blueCoralStation1 = CORAL_STATION_1;
            Pose2d blueCoralStation2 = CORAL_STATION_2;

            // Blue Coral Stations
            add(blueCoralStation1);
            add(blueCoralStation2);

            // Red Coral Stations
            add(PoseCalculations.flipPose(blueCoralStation1));
            add(PoseCalculations.flipPose(blueCoralStation2));
        }};

        public static final List<Pose2d> GET_CORAL_STATION_POSITIONS() {
            int startIndex = Robot.isRedAlliance() ? 2 : 0;
            return CORAL_STATION_POSITIONS.subList(startIndex, startIndex + 2);
        }

        public static final Pose2d BLUE_REEF = new Pose2d(4.477431, FieldConstants.FIELD_MAX_HEIGHT / 2.0, new Rotation2d());

        public static final List<Pose2d> REEF_POSITIONS = new ArrayList<Pose2d>() {{
            add(BLUE_REEF);
            add(PoseCalculations.flipPose(BLUE_REEF));
        }};

        public static final Pose2d GET_REEF_POSITION() {
            return REEF_POSITIONS.get(Robot.isRedAlliance() ? 1 : 0);
        }

        public static final double POLE_DISTANCE = 0.1643085;

        // Centers of each reef side, letters are ordered from driver station POV (left than right)
        public static final Pose2d BLUE_REEF_AB = new Pose2d(3.645757, FieldConstants.FIELD_MAX_HEIGHT / 2.0, Rotation2d.fromDegrees(180));
        public static final Pose2d BLUE_REEF_CD = new Pose2d(4.061614, 3.305661, Rotation2d.fromDegrees(240));
        public static final Pose2d BLUE_REEF_FE = new Pose2d(4.893287, 3.305684, Rotation2d.fromDegrees(300));
        public static final Pose2d BLUE_REEF_HG = new Pose2d(5.309104, FieldConstants.FIELD_MAX_HEIGHT / 2.0, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_REEF_JI = new Pose2d(4.893247, 4.746185, Rotation2d.fromDegrees(60));
        public static final Pose2d BLUE_REEF_KL = new Pose2d(4.061574, 4.746162, Rotation2d.fromDegrees(120));

        public static final Pose2d BLUE_REEF_A = new Pose2d(BLUE_REEF_AB.getX() + POLE_DISTANCE * Math.cos(Math.toRadians(180-90)), BLUE_REEF_AB.getY() + POLE_DISTANCE * Math.sin(Math.toRadians(180-90)), Rotation2d.fromDegrees(180));
        public static final Pose2d BLUE_REEF_B = new Pose2d(BLUE_REEF_AB.getX() + POLE_DISTANCE * Math.cos(Math.toRadians(180-90)), BLUE_REEF_AB.getY() - POLE_DISTANCE * Math.sin(Math.toRadians(180-90)), Rotation2d.fromDegrees(180));
        public static final Pose2d BLUE_REEF_C = new Pose2d(BLUE_REEF_CD.getX() + POLE_DISTANCE * Math.cos(Math.toRadians(240-90)), BLUE_REEF_CD.getY() + POLE_DISTANCE * Math.sin(Math.toRadians(240-90)), Rotation2d.fromDegrees(240));
        public static final Pose2d BLUE_REEF_D = new Pose2d(BLUE_REEF_CD.getX() - POLE_DISTANCE * Math.cos(Math.toRadians(240-90)), BLUE_REEF_CD.getY() - POLE_DISTANCE * Math.sin(Math.toRadians(240-90)), Rotation2d.fromDegrees(240));
        public static final Pose2d BLUE_REEF_E = new Pose2d(BLUE_REEF_FE.getX() + POLE_DISTANCE * Math.cos(Math.toRadians(300-90)), BLUE_REEF_FE.getY() + POLE_DISTANCE * Math.sin(Math.toRadians(300-90)), Rotation2d.fromDegrees(300));
        public static final Pose2d BLUE_REEF_F = new Pose2d(BLUE_REEF_FE.getX() - POLE_DISTANCE * Math.cos(Math.toRadians(300-90)), BLUE_REEF_FE.getY() - POLE_DISTANCE * Math.sin(Math.toRadians(300-90)), Rotation2d.fromDegrees(300));
        public static final Pose2d BLUE_REEF_G = new Pose2d(BLUE_REEF_HG.getX() + POLE_DISTANCE * Math.cos(Math.toRadians(0-90)), BLUE_REEF_HG.getY() + POLE_DISTANCE * Math.sin(Math.toRadians(0-90)), Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_REEF_H = new Pose2d(BLUE_REEF_HG.getX() + POLE_DISTANCE * Math.cos(Math.toRadians(0-90)), BLUE_REEF_HG.getY() - POLE_DISTANCE * Math.sin(Math.toRadians(0-90)), Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_REEF_I = new Pose2d(BLUE_REEF_JI.getX() + POLE_DISTANCE * Math.cos(Math.toRadians(60-90)), BLUE_REEF_JI.getY() + POLE_DISTANCE * Math.sin(Math.toRadians(60-90)), Rotation2d.fromDegrees(60));
        public static final Pose2d BLUE_REEF_J = new Pose2d(BLUE_REEF_JI.getX() - POLE_DISTANCE * Math.cos(Math.toRadians(60-90)), BLUE_REEF_JI.getY() - POLE_DISTANCE * Math.sin(Math.toRadians(60-90)), Rotation2d.fromDegrees(60));
        public static final Pose2d BLUE_REEF_K = new Pose2d(BLUE_REEF_KL.getX() + POLE_DISTANCE * Math.cos(Math.toRadians(120-90)), BLUE_REEF_KL.getY() + POLE_DISTANCE * Math.sin(Math.toRadians(120-90)), Rotation2d.fromDegrees(120));
        public static final Pose2d BLUE_REEF_L = new Pose2d(BLUE_REEF_KL.getX() - POLE_DISTANCE * Math.cos(Math.toRadians(120-90)), BLUE_REEF_KL.getY() - POLE_DISTANCE * Math.sin(Math.toRadians(120-90)), Rotation2d.fromDegrees(120));

        public static final List<ReefSide> REEF_FACE_POSITIONS = new ArrayList<ReefSide>() {{
            // All points are in meters and radians
            // All relative to the blue origin
            // Positions go from blueReef1 (the farthest from blue driverstation) clockwise around
            // even = high, odd = low
            ReefSide blueReef6 = new ReefSide(BLUE_REEF_KL, BLUE_REEF_K, BLUE_REEF_L, 19);
            ReefSide blueReef5 = new ReefSide(BLUE_REEF_JI, BLUE_REEF_J, BLUE_REEF_I, 20);
            ReefSide blueReef4 = new ReefSide(BLUE_REEF_HG, BLUE_REEF_H, BLUE_REEF_G, 21);
            ReefSide blueReef3 = new ReefSide(BLUE_REEF_FE, BLUE_REEF_F, BLUE_REEF_E, 22);
            ReefSide blueReef2 = new ReefSide(BLUE_REEF_CD, BLUE_REEF_C, BLUE_REEF_D, 17);
            ReefSide blueReef1 = new ReefSide(BLUE_REEF_AB, BLUE_REEF_A, BLUE_REEF_B, 18);

            // Blue Reef
            add(blueReef1);
            add(blueReef2);
            add(blueReef3);
            add(blueReef4);
            add(blueReef5);
            add(blueReef6);

            // Red Reef
            add(blueReef1.otherAlliance(7)); 
            add(blueReef2.otherAlliance(8));
            add(blueReef3.otherAlliance(9));
            add(blueReef4.otherAlliance(10));
            add(blueReef5.otherAlliance(11));
            add(blueReef6.otherAlliance(6));
        }};

        public static final List<ReefSide> GET_REEF_FACE_POSITIONS() {
            int startIndex = Robot.isRedAlliance() ? 6 : 0;
            return REEF_FACE_POSITIONS.subList(startIndex, startIndex + 6);
        }

        public static final Pose2d CAGE_1 = new Pose2d(8.77, 7.26, Rotation2d.fromDegrees(270));
        public static final Pose2d CAGE_2 = new Pose2d(8.77, 6.16, Rotation2d.fromDegrees(270));
        public static final Pose2d CAGE_3 = new Pose2d(8.77, 5.07, Rotation2d.fromDegrees(270));
        
        public static final List<Pose2d> CAGE_POSITIONS = new ArrayList<Pose2d>() {{
            // blueCage1 starts as the highest from field origin and goes down from there

            Pose2d blueCage1 = CAGE_1;
            Pose2d blueCage2 = CAGE_2;
            Pose2d blueCage3 = CAGE_3;

            // Blue Cages
            add(blueCage1);
            add(blueCage2);
            add(blueCage3);

            // Red Cages
            add(PoseCalculations.mirrorPose(blueCage1));
            add(PoseCalculations.mirrorPose(blueCage2));
            add(PoseCalculations.mirrorPose(blueCage3));
        }};


        public static final List<Pose2d> GET_CAGE_POSITIONS() {
            int startIndex = Robot.isRedAlliance() ? 3 : 0;
            return CAGE_POSITIONS.subList(startIndex, startIndex + 3);
        }

        public static final double REEF_HEIGHT_L1 = 0.40;
        public static final double REEF_HEIGHT_L2 = 0.77;
        public static final double REEF_HEIGHT_L3 = 1.18;
        public static final double REEF_HEIGHT_L4 = 1.82;

        public static final Pose2d STAGED_TREE_1 = new Pose2d(1.21, 5.86, Rotation2d.fromDegrees(0));
        public static final Pose2d STAGED_TREE_2 = new Pose2d(1.21, 4.03, Rotation2d.fromDegrees(0));
        public static final Pose2d STAGED_TREE_3 = new Pose2d(1.21, 2.20, Rotation2d.fromDegrees(0));

        public static final List<Pose2d> STAGED_POSITIONS = new ArrayList<Pose2d>() {{

            Pose2d blueStagedTree1 = STAGED_TREE_1;
            Pose2d blueStagedTree2 = STAGED_TREE_2;
            Pose2d blueStagedTree3 = STAGED_TREE_3;

            // Blue Staged Trees
            add(blueStagedTree1);
            add(blueStagedTree2);
            add(blueStagedTree3);

            // Red Staged Trees
            add(PoseCalculations.mirrorPose(blueStagedTree1));
            add(PoseCalculations.mirrorPose(blueStagedTree2));
            add(PoseCalculations.mirrorPose(blueStagedTree3));
        }};

        public static final List<Pose2d> GET_STAGED_POSITIONS() {
            int startIndex = Robot.isRedAlliance() ? 3 : 0;
            return STAGED_POSITIONS.subList(startIndex, startIndex + 3);
        }

        public static final Pose2d BLUE_RESET_ODO_POSITION = new Pose2d(3.23, 4.026, new Rotation2d(Math.PI));

        public static final List<Pose2d> RESET_ODO_POSITIONS = new ArrayList<Pose2d>() {{
            add(BLUE_RESET_ODO_POSITION);
            add(PoseCalculations.flipPose(BLUE_RESET_ODO_POSITION));
        }};

        public static final Pose2d GET_RESET_ODO_POSITION() {
            return RESET_ODO_POSITIONS.get(Robot.isRedAlliance() ? 1 : 0);
        }

        public static final double NEAR_REEF_METERS = 1.4; 

        // D:
        public static final int[] VALID_TAGS = new int[] { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22 }; // D:


        public static final double CORAL_RADIUS_METERS = 0.0507745;
        public static final double ALGAE_RADIUS_METERS = 0.205;
        static final Translation3d REEF_CENTER = new Translation3d(4.49, 4.0225, 0);
        static final double HEXAGON_RADS = Units.degreesToRadians(360/6);
                
        private static Pose3d rotatePose(Pose3d pose, double angle) {
            return pose.rotateAround(REEF_CENTER, new Rotation3d(0, 0, angle));
        }
        
        static final Rotation3d L2_BASE_ANGLE = new Rotation3d(0, Units.degreesToRadians(35), 0);
        static final Pose3d L2_POSE_0 = new Pose3d(3.775, 3.855, 0.739, L2_BASE_ANGLE);
        static final Pose3d L2_POSE_1 = new Pose3d(3.775, 4.19, 0.739, L2_BASE_ANGLE);
        
        public static final Pose3d[] L2_CORAL_PLACEMENT_POSITIONS = createCoralPlacementPositions(L2_POSE_0, L2_POSE_1);

        static final double L2_L3_HEIGHT = 0.4031;
        static final Pose3d L3_POSE_0 = new Pose3d(L2_POSE_0.getX(), L2_POSE_0.getY(), L2_POSE_0.getZ()+L2_L3_HEIGHT, L2_BASE_ANGLE);
        static final Pose3d L3_POSE_1 = new Pose3d(L2_POSE_1.getX(), L2_POSE_1.getY(), L2_POSE_1.getZ()+L2_L3_HEIGHT, L2_BASE_ANGLE);
        public static final Pose3d[] L3_CORAL_PLACEMENT_POSITIONS = createCoralPlacementPositions(L3_POSE_0, L3_POSE_1);

        static final Rotation3d L4_BASE_ANGLE = new Rotation3d(0, Units.degreesToRadians(90), 0);
        static final double L3_L4_HEIGHT = 0.616;
        static final Pose3d L4_POSE_0 = new Pose3d(L3_POSE_0.getX()-4*CORAL_RADIUS_METERS/3, L3_POSE_0.getY(), L3_POSE_0.getZ()+L3_L4_HEIGHT, L4_BASE_ANGLE);
        static final Pose3d L4_POSE_1 = new Pose3d(L3_POSE_1.getX()-4*CORAL_RADIUS_METERS/3, L3_POSE_1.getY(), L3_POSE_1.getZ()+L3_L4_HEIGHT, L4_BASE_ANGLE);
        public static final Pose3d[] L4_CORAL_PLACEMENT_POSITIONS = createCoralPlacementPositions(L4_POSE_0, L4_POSE_1);

        public static final Pose3d[] BLUE_CORAL_PLACEMENT_POSITIONS = combineCoralPlacementPositions(
            L2_CORAL_PLACEMENT_POSITIONS, L3_CORAL_PLACEMENT_POSITIONS, L4_CORAL_PLACEMENT_POSITIONS
        );

        public static final Pose3d[] CORAL_PLACEMENT_POSITIONS = combineCoralPlacementPositions(
            BLUE_CORAL_PLACEMENT_POSITIONS, flipCoralPlacementPositions(BLUE_CORAL_PLACEMENT_POSITIONS)
        );

        private static Pose3d[] createCoralPlacementPositions(Pose3d pose0, Pose3d pose1) {
            return new Pose3d[] {
                pose0,
                pose1,
                rotatePose(pose0, HEXAGON_RADS),
                rotatePose(pose1, HEXAGON_RADS),
                rotatePose(pose0, 2 * HEXAGON_RADS),
                rotatePose(pose1, 2 * HEXAGON_RADS),
                rotatePose(pose0, 3 * HEXAGON_RADS),
                rotatePose(pose1, 3 * HEXAGON_RADS),
                rotatePose(pose0, 4 * HEXAGON_RADS),
                rotatePose(pose1, 4 * HEXAGON_RADS),
                rotatePose(pose0, 5 * HEXAGON_RADS),
                rotatePose(pose1, 5 * HEXAGON_RADS),
            };
        }

        private static Pose3d[] combineCoralPlacementPositions(Pose3d[]... positionsArrays) {
            int totalLength = 0;
            for (Pose3d[] positions : positionsArrays) {
                totalLength += positions.length;
            }

            Pose3d[] combined = new Pose3d[totalLength];
            int currentIndex = 0;
            for (Pose3d[] positions : positionsArrays) {
                System.arraycopy(positions, 0, combined, currentIndex, positions.length);
                currentIndex += positions.length;
            }

            return combined;
        }

        private static Pose3d[] flipCoralPlacementPositions(Pose3d[] positions) {
            Pose3d[] flipped = new Pose3d[positions.length];
            for (int i = 0; i < positions.length; i++) {
                flipped[i] = PoseCalculations.flipPose3d(positions[i]);
            }
            return flipped;
        }

        /// 
        /// ALGAE LAND
        /// 
        static final Pose3d BLUE_POSE_HIGH = new Pose3d(3.81, 4.025, 1.315, new Rotation3d());
        static final Pose3d BLUE_POSE_LOW = new Pose3d(3.81, 4.025, .905, new Rotation3d());
        static final Pose3d[] BLUE_ALGAE_LOCATIONS = new Pose3d[] {
            BLUE_POSE_HIGH,
            rotatePose(BLUE_POSE_HIGH, HEXAGON_RADS*2),
            rotatePose(BLUE_POSE_HIGH, HEXAGON_RADS*4),
            rotatePose(BLUE_POSE_LOW, HEXAGON_RADS),
            rotatePose(BLUE_POSE_LOW, HEXAGON_RADS*3),
            rotatePose(BLUE_POSE_LOW, HEXAGON_RADS*5)
        };

        public static final List<Pose3d> ALGAE_REMOVAL_LOCATIONS_LIST = new ArrayList<Pose3d>() {{
            addAll(Arrays.asList(BLUE_ALGAE_LOCATIONS));
            addAll(Arrays.asList(flipCoralPlacementPositions(BLUE_ALGAE_LOCATIONS)));
        }};
        
        public static final Pose3d[] ALGAE_REMOVAL_LOCATIONS_ARRAY = ALGAE_REMOVAL_LOCATIONS_LIST.toArray(new Pose3d[0]);
    }
}