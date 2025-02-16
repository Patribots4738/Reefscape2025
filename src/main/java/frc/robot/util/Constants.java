// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
            if (!FieldConstants.IS_SIMULATION && robotType == RobotType.SIMBOT) {
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
            0.2655697,
            0.0933069,
            0.2824708
        );

        public static final Translation3d CLIMB_OFFSET = new Translation3d(
            ROBOT_OFFSET.getX() - 0.215, 
            ROBOT_OFFSET.getY() + 0.1442, 
            ROBOT_OFFSET.getZ() - 0.0946
        );

        public static final Translation3d WRIST_OFFSET = new Translation3d(
            ROBOT_OFFSET.getX() - 0.4368, 
            ROBOT_OFFSET.getY() - 0.0933069, 
            ROBOT_OFFSET.getZ() + 0.504
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
        public static final double BUMPER_LENGTH_METERS = Units.inchesToMeters(2.75);

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

        public static final String REEF_NODES = "ABCDEFGHIJKL";
        
        // Below is gotten from choreo
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Units.degreesToRadians(1137.21);
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Units.degreesToRadians(792.90);

        public static final double HDC_POSITION_TOLERANCE_METERS = Units.inchesToMeters(1);
        public static final double HDC_ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(2);


        public static final GainConstants AUTO_XY_GAINS = new GainConstants(
            9.0, 
            0.0, 
            0.2
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
            3.725, 
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
                AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED)) 
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
            "2-J4-CS1-K4-CS1-L4-CS1-A4-CS1",
            "1-K4-CS1-L4-CS1-A4-CS1-B4-CS1",
            "2-I4-CS1-J4-CS1-K4-CS1-L4-CS1",
            "4-H4-CS2-G4-CS2-B4-CS2-A4-CS2",
            "6-E4-CS2-D4-CS2-C4-CS2-B4-CS2"


        };

        public static final double REEF_ALIGNMENT_MULTIPLIER = 0.5;
        public static final double CAGE_ALIGNMENT_MULTIPLIER = 0.5;

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

        public static final double FRONT_LEFT_TURN_ENCODER_OFFSET = 0.280518;
        public static final double FRONT_RIGHT_TURN_ENCODER_OFFSET = -0.283203;
        public static final double REAR_LEFT_TURN_ENCODER_OFFSET = 0.081787;
        public static final double REAR_RIGHT_TURN_ENCODER_OFFSET = 0.114990;

        public static final double TURNING_MOTOR_REDUCTION = 12.8;

        public static final double MAX_TURNING_MOTOR_VELOCITY_RADIANS_PER_SEC = 609.669;

        public static final boolean INVERT_TURNING_MOTOR = false;

        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = KrakenMotorConstants.KRAKENX60_FREE_SPEED_RPM_FOC / 60;
        // **********************************************************************MK4c SWERVE**********************
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(1.9548910849480585 * 2);
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


        // public static final double DRIVING_P = 20d;
        // public static final double DRIVING_I = 0;
        // public static final double DRIVING_D = 0.0;
        // public static final double DRIVING_S = 0.51215;
        // public static final double DRIVING_V = 0.12199;

        public static final GainConstants DRIVING_GAINS = new GainConstants(
            DRIVING_P,
            DRIVING_I,
            DRIVING_D,
            DRIVING_S,
            DRIVING_V,
            0.0
        );

        public static final LoggedGainConstants LOGGED_DRIVING_GAINS = new LoggedGainConstants(MK4cSwerveModuleConstants.DRIVING_GAINS, "Swerve/Drive");

        public static final double TURNING_P = 1500d;
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 30d;
        public static final double TURNING_S = 2.800419999999998;

        // public static final double TURNING_P = 5500d;
        // public static final double TURNING_I = 0.0;
        // public static final double TURNING_D = 510d;
        // public static final double TURNING_S = 1.0392280000000027;

        public static final GainConstants TURNING_GAINS = new GainConstants(
            TURNING_P,
            TURNING_I,
            TURNING_D,
            TURNING_S,
            0.0,
            0.0
        );

        public static final LoggedGainConstants LOGGED_TURNING_GAINS = new LoggedGainConstants(MK4cSwerveModuleConstants.TURNING_GAINS, "Swerve/Turn");

    }

    public static final class CoralClawConstants {

        public static final int CAN_ID = 9;

        public static final boolean BRAKE_MOTOR = true;
        public static final double CURRENT_LIMIT = 80.0;

        public static final boolean MOTOR_INVERTED = false;

        public static final double INTAKE_PERCENT = 0.35;
        public static final double OUTTAKE_PERCENT = -0.75;

        public static final double PLACING_NAMED_COMMAND_TIME = 0.5;

        public static final double CURRENT_THRESHOLD_HAS_PIECE_AMPS = 5.0;

        public static final double CORAL_CLAW_CURRENT_DEADBAND = 10.101205;

    }

    public static final class AlgaeClawConstants {

        public static final int CAN_ID = 14;

        public static final boolean BRAKE_MOTOR = true;
        public static final double CURRENT_LIMIT = 80-.0;

        public static final boolean MOTOR_INVERTED = true;

        public static final double INTAKE_PERCENT = 0.35;
        public static final double OUTTAKE_PERCENT = -0.35;

        public static final double PLACING_NAMED_COMMAND_TIME = 0.5;

    }
    public static final class ElevatorConstants {

        public static final int LEADER_CAN_ID = 10;
        public static final int FOLLOWER_CAN_ID = 11;

        public static final boolean BRAKE_MOTOR = true;
        public static final double CURRENT_LIMIT = 80.0;

        public static final boolean MOTOR_INVERTED = false;

        public static final double GEAR_RATIO = 16.0;
        public static final double MAX_DISPLACEMENT_METERS = 0.548;
        public static final double POSITION_CONVERSION_FACTOR = Units.inchesToMeters(6.01716904509);
        public static final double VELOCITY_CONVERSION_FACTOR = Units.inchesToMeters(6.01716904509);

        public static final double P = !FieldConstants.IS_SIMULATION ? 180d : 750;
        public static final double I = !FieldConstants.IS_SIMULATION ? 0d : 0d;
        public static final double D = !FieldConstants.IS_SIMULATION ? 20d : 585;
        public static final double A = !FieldConstants.IS_SIMULATION ? 0d : 0d;
        public static final double S = !FieldConstants.IS_SIMULATION ? 0d : 0d;
        public static final double G = !FieldConstants.IS_SIMULATION ? 0d : 0d;

        public static final GainConstants GAINS = new GainConstants(
            P,
            I,
            D,
            A,
            S,
            0.0,
            G
        );

        public static final LoggedGainConstants LOGGED_GAINS = new LoggedGainConstants(ElevatorConstants.GAINS, "Elevator");
        
        public static final double VELOCITY = 1d;
        public static final double ACCELERATION = 2d;
        public static final double JERK = 0d;

        public static final double STOW_POSITION_METERS = 0.0;
        public static final double INTAKE_POSITION_METERS = 0.0;
        public static final double L1_POSITION_METERS = 0.0;
        public static final double L2_POSITION_METERS = 0.12;
        public static final double L3_POSITION_METERS = 0.32;
        public static final double L4_POSITION_METERS = MAX_DISPLACEMENT_METERS;
        public static final double L3_POSITION_REMOVE_ALGAE = 0.18;
        public static final double L2_POSITION_REMOVE_ALGAE = 0.0;

        public static final double DEADBAND_METERS = 0.02;

    }

    public static final class WristConstants {

        public static final int CAN_ID = 12;

        public static final double ENCODER_POSITION_OFFSET_ROTATIONS = 0.0;
        public static final boolean ENCODER_INVERTED = false;

        public static final boolean BRAKE_MOTOR = true;

        public static final boolean MOTOR_INVERTED = false;

        public static final double P = !FieldConstants.IS_SIMULATION ? 1500d : 2500d;
        public static final double I = !FieldConstants.IS_SIMULATION ? 0d : 0d;
        public static final double D = !FieldConstants.IS_SIMULATION ? 100d : 800d;
        public static final double A = !FieldConstants.IS_SIMULATION ? 0d : 0d;
        public static final double S = !FieldConstants.IS_SIMULATION ? 0d : 0d;
        public static final double G = !FieldConstants.IS_SIMULATION ? 0d : 0d;

        public static final GainConstants GAINS = new GainConstants(
            P,
            I,
            D,
            A,
            S,
            0.0,
            G
        );

        public static final LoggedGainConstants LOGGED_GAINS = new LoggedGainConstants(WristConstants.GAINS, "Wrist");

        public static final double VELOCITY = 10d;
        public static final double ACCELERATION = !FieldConstants.IS_SIMULATION ? 20d : 15d;
        public static final double JERK = 0d;

        public static final double GEAR_RATIO = 60.0;
      
        public static final double POSITION_CONVERSION_FACTOR = 2 * Math.PI;
        public static final double VELOCITY_CONVERSION_FACTOR = 2 * Math.PI;
        public static final double ENCODER_POSITION_CONVERSION_FACTOR = 2 * Math.PI;

        public static final double CURRENT_LIMIT = 60.0;

        public static final double CG_OFFSET_ANGLE_RADIANS = 0.187;

        public static final double MIN_ANGLE_RADIANS = -1.08231258091;
        public static final double MAX_ANGLE_RADIANS = 2.95;

        public static final double RESET_ANGLE_RADIANS = 2.95;

        public static final double CLIMB_RADIANS = 1.7;
        public static final double TRANSITION_RADIANS = 1.7;

        public static final double DEADBAND_RADIANS = 0.06;
        public static final double STOW_POSITION_RADIANS = 0.07;
        public static final double INTAKE_POSITION_RADIANS = -0.18;
        public static final double L1_POSITION_RADIANS = 2.8;
        public static final double L2_POSITION_RADIANS = 2.2;
        public static final double L3_POSITION_RADIANS = 2.22;
        public static final double L4_POSITION_RADIANS = 2.65;
        public static final double ALGAE_REMOVAL = 1.57;


    }

    public static final class ClimbConstants {

        public static final int CAN_ID = 13;

        public static final boolean MOTOR_INVERTED = false;

        public static final double GEAR_RATIO = 233 + 1d/3d;

        public static final double POSITION_CONVERSION_FACTOR = 2.0 * Math.PI;
        public static final double VELOCITY_CONVERSION_FACTOR = 2.0 * Math.PI;

        public static final double CURRENT_LIMIT = 120.0;
        public static final boolean BRAKE_MOTOR = true;

        public static final double MIN_ANGLE_RADIANS = 0.0;
        public static final double MAX_ANGLE_RADIANS = 1.5;

        public static final double P = !FieldConstants.IS_SIMULATION ? 25d : 0d;
        public static final double I = !FieldConstants.IS_SIMULATION ? 0d : 0d;
        public static final double D = !FieldConstants.IS_SIMULATION ? 2d : 0d;
        public static final double A = !FieldConstants.IS_SIMULATION ? 0d : 0d;
        public static final double S = !FieldConstants.IS_SIMULATION ? 0d : 0d; 
        public static final double G = !FieldConstants.IS_SIMULATION ? 0d : 0d;

        public static final double VELOCITY = 0.0;
        public static final double ACCELERATION = 0.0;
        public static final double JERK = 0.0;

        public static final GainConstants GAINS = new GainConstants(
            P,
            I,
            D,
            A,
            S,
            G
        );

        public static final LoggedGainConstants LOGGED_GAINS = new LoggedGainConstants(ClimbConstants.GAINS, "Climb");
        
        public static final double STOW_POSITION_RADIANS = 0.0;
        public static final double READY_POSITION_RADIANS = 1.5;
        public static final double FINAL_POSITION_RADIANS = 0.169;

        public static final double Y_CHASSIS_OFFSET = 0.0508;

        public static final double DEADBAND_RADIANS = Units.degreesToRadians(3.0);

    }

    public static final class OIConstants {

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final double DRIVER_DEADBAND = 0.15;
        public static final double OPERATOR_DEADBAND = 0.15;

        // See https://www.desmos.com/calculator/e07raajzh5
        // And
        // https://docs.google.com/spreadsheets/d/1Lytrh6q9jkz4u1gmF1Sk8kTpj8DxW-uwRE_QMnTt8Lk
        public static final double CONTROLLER_CORNER_SLOPE_1 = 1 / 0.7;
        public static final double CONTROLLER_CORNER_SLOPE_2 = 0.7;

        public enum DriverMode {
            DOUBLE,
            DEV
        }

        public static final DriverMode DRIVER_MODE = DriverMode.DEV;

    }

    public static final class CameraConstants {

        

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

        public static final double TALONFX_FAST_UPDATE_FREQ_HZ = 100;// TODO: FIND THE SWEET SPOT
        public static final double TALONFX_MID_UPDATE_FREQ_HZ = 50; // TODO: FIND THE SWEET SPOT
        public static final double TALONFX_SLOW_UPDATE_FREQ_HZ = 4; // TODO: FIND THE SWEET SPOT

        public static final HashMap<Integer, Kraken> KRAKEN_MOTOR_MAP = new HashMap<Integer, Kraken>();

        public static final HashMap<String, List<Kraken>> KRAKEN_MOTOR_GROUPS = new HashMap<String, List<Kraken>>();

        public static Map<String, List<Kraken>> initializeMotorGroupMap() {
            KRAKEN_MOTOR_GROUPS.put("Drive", new ArrayList<Kraken>() {{
                add(KRAKEN_MOTOR_MAP.get(MK4cSwerveModuleConstants.FRONT_LEFT_DRIVING_CAN_ID));
                add(KRAKEN_MOTOR_MAP.get(MK4cSwerveModuleConstants.FRONT_RIGHT_DRIVING_CAN_ID));
                add(KRAKEN_MOTOR_MAP.get(MK4cSwerveModuleConstants.REAR_LEFT_DRIVING_CAN_ID));
                add(KRAKEN_MOTOR_MAP.get(MK4cSwerveModuleConstants.REAR_RIGHT_DRIVING_CAN_ID));
            }});
            KRAKEN_MOTOR_GROUPS.put("Turn", new ArrayList<Kraken>() {{
                add(KRAKEN_MOTOR_MAP.get(MK4cSwerveModuleConstants.FRONT_LEFT_TURNING_CAN_ID));
                add(KRAKEN_MOTOR_MAP.get(MK4cSwerveModuleConstants.FRONT_RIGHT_TURNING_CAN_ID));
                add(KRAKEN_MOTOR_MAP.get(MK4cSwerveModuleConstants.REAR_LEFT_TURNING_CAN_ID));
                add(KRAKEN_MOTOR_MAP.get(MK4cSwerveModuleConstants.REAR_RIGHT_TURNING_CAN_ID));
            }});
            KRAKEN_MOTOR_GROUPS.put("Elevator", new ArrayList<Kraken>() {{
                add(KRAKEN_MOTOR_MAP.get(ElevatorConstants.LEADER_CAN_ID));
                add(KRAKEN_MOTOR_MAP.get(ElevatorConstants.FOLLOWER_CAN_ID));
            }});
            KRAKEN_MOTOR_GROUPS.put("Wrist", new ArrayList<Kraken>() {{
                add(KRAKEN_MOTOR_MAP.get(WristConstants.CAN_ID));
            }});
            KRAKEN_MOTOR_GROUPS.put("Climb", new ArrayList<Kraken>() {{
                add(KRAKEN_MOTOR_MAP.get(ClimbConstants.CAN_ID));
            }});

            return KRAKEN_MOTOR_GROUPS;
        }

    }

    public static final class CANCoderConstants {

        public static final double ENCODER_UPDATE_FREQ_HZ = 250; // TODO: FIND THE SWEET SPOT

    }

    public static final class PigeonConstants {

        public static final double PIGEON_FAST_UPDATE_FREQ_HZ = 250; // TODO: FIND THE SWEET SPOT

    }

    public static final class GeneralHardwareConstants {
        public static final boolean SAFE_HARDWARE_MODE = false;
    }

    public static final class FieldConstants {

        public static final boolean IS_SIMULATION = Robot.isSimulation();


        public static final double FIELD_MAX_LENGTH = 17.55;
        // 2d height
        public static final double FIELD_MAX_HEIGHT = 8.0518;

        // All These Positions Are For The Blue Side Unless Specified Otherwise

        public static final Pose2d BLUE_PROCESSER = new Pose2d(6.00, 0.00, Rotation2d.fromDegrees(0));

        public static final List<Pose2d> PROCESSER_POSITIONS = new ArrayList<Pose2d>() {{
            // Blue Processer
            add(BLUE_PROCESSER);
            // Red Processer
            add(PoseCalculations.mirrorPose(BLUE_PROCESSER));
        }};

        public static final Pose2d GET_PROCESSER_POSITION() {
            int index = Robot.isRedAlliance() ? 1 : 0;
            return PROCESSER_POSITIONS.get(index);
        }


        public static final Pose2d CORAL_STATION_1 = new Pose2d(0.82, 7.39, Rotation2d.fromDegrees(125));
        public static final Pose2d CORAL_STATION_2 = new Pose2d(0.82, 0.65, Rotation2d.fromDegrees(-125));

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

        public static final Pose2d BLUE_REEF = new Pose2d(4.508, FieldConstants.FIELD_MAX_HEIGHT / 2.0, new Rotation2d());

        public static final List<Pose2d> REEF_POSITIONS = new ArrayList<Pose2d>() {{
            add(BLUE_REEF);
            add(PoseCalculations.flipPose(BLUE_REEF));
        }};

        public static final Pose2d GET_REEF_POSITION() {
            return REEF_POSITIONS.get(Robot.isRedAlliance() ? 1 : 0);
        }

        public static final Pose2d BLUE_REEF_A = new Pose2d(3.72, 4.19, Rotation2d.fromDegrees(180));
        public static final Pose2d BLUE_REEF_B = new Pose2d(3.72, 3.86, Rotation2d.fromDegrees(180));
        public static final Pose2d BLUE_REEF_C = new Pose2d(3.94, 3.44, Rotation2d.fromDegrees(240));
        public static final Pose2d BLUE_REEF_D = new Pose2d(4.24, 3.27, Rotation2d.fromDegrees(240));
        public static final Pose2d BLUE_REEF_E = new Pose2d(4.73, 3.27, Rotation2d.fromDegrees(300));
        public static final Pose2d BLUE_REEF_F = new Pose2d(5.02, 3.44, Rotation2d.fromDegrees(300));
        public static final Pose2d BLUE_REEF_G = new Pose2d(5.26, 3.86, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_REEF_H = new Pose2d(5.26, 4.19, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_REEF_I = new Pose2d(5.02, 4.61, Rotation2d.fromDegrees(60));
        public static final Pose2d BLUE_REEF_J = new Pose2d(4.73, 4.78, Rotation2d.fromDegrees(60));
        public static final Pose2d BLUE_REEF_K = new Pose2d(4.24, 4.78, Rotation2d.fromDegrees(120));
        public static final Pose2d BLUE_REEF_L = new Pose2d(3.94, 4.61, Rotation2d.fromDegrees(120));

        // Centers of each reef side, letters are ordered from driver station POV (left than right)
        public static final Pose2d BLUE_REEF_AB = new Pose2d(3.67, FieldConstants.FIELD_MAX_HEIGHT / 2.0, Rotation2d.fromDegrees(180));
        public static final Pose2d BLUE_REEF_CD = new Pose2d(4.09, 3.37, Rotation2d.fromDegrees(240));
        public static final Pose2d BLUE_REEF_FE = new Pose2d(4.91, 3.34, Rotation2d.fromDegrees(300));
        public static final Pose2d BLUE_REEF_HG = new Pose2d(5.30, FieldConstants.FIELD_MAX_HEIGHT / 2.0, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_REEF_JI = new Pose2d(4.88, 4.74, Rotation2d.fromDegrees(60));
        public static final Pose2d BLUE_REEF_KL = new Pose2d(4.06, 4.72, Rotation2d.fromDegrees(120));

        public static final List<ReefSide> REEF_FACE_POSITIONS = new ArrayList<ReefSide>() {{
            // All points are in meters and radians
            // All relative to the blue origin
            // Positions go from blueReef1 (the farthest from blue driverstation) clockwise around
            // even = high, odd = low
            ReefSide blueReef6 = new ReefSide(BLUE_REEF_KL, BLUE_REEF_K, BLUE_REEF_L);
            ReefSide blueReef5 = new ReefSide(BLUE_REEF_JI, BLUE_REEF_J, BLUE_REEF_I);
            ReefSide blueReef4 = new ReefSide(BLUE_REEF_HG, BLUE_REEF_H, BLUE_REEF_G);
            ReefSide blueReef3 = new ReefSide(BLUE_REEF_FE, BLUE_REEF_F, BLUE_REEF_E);
            ReefSide blueReef2 = new ReefSide(BLUE_REEF_CD, BLUE_REEF_C, BLUE_REEF_D);
            ReefSide blueReef1 = new ReefSide(BLUE_REEF_AB, BLUE_REEF_A, BLUE_REEF_B);

            // Blue Reef
            add(blueReef1);
            add(blueReef2);
            add(blueReef3);
            add(blueReef4);
            add(blueReef5);
            add(blueReef6);

            // Red Reef
            add(blueReef1.otherAlliance()); 
            add(blueReef2.otherAlliance());
            add(blueReef3.otherAlliance());
            add(blueReef4.otherAlliance());
            add(blueReef5.otherAlliance());
            add(blueReef6.otherAlliance());
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

    }

}