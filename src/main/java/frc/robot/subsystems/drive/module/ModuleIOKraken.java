package frc.robot.subsystems.drive.module;

import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.MK4cSwerveModuleConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.CANCoderCustom;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;
import frc.robot.util.hardware.phoenix.Kraken.TelemetryPreference;

public class ModuleIOKraken implements ModuleIO {

    private final Kraken driveMotor;
    private final Kraken turnMotor;
    private final CANCoderCustom turnEncoder;

    /**
     * Creates new MK4c swerve module.
     * 
     * @param drivingCANId CAN ID of the driving motor
     * @param turningCANId CAN ID of the turning motor
     * @param canCoderId CAN ID of the modules encoder
     * @param chassisAngularOffset angular offset of module to the chasis
     * @param index
     */
    public ModuleIOKraken(int drivingCANId, int turningCANId, int canCoderId, double absoluteEncoderOffset) {
        driveMotor = new Kraken(drivingCANId, "Drivebase", true, true, ControlPreference.TORQUE_CURRENT);
        turnMotor = new Kraken(turningCANId, "Drivebase", true, true, ControlPreference.TORQUE_CURRENT);
        turnEncoder = new CANCoderCustom(canCoderId, "Drivebase");
        resetDriveEncoder();
        configEncoder(absoluteEncoderOffset);
        configMotors();
    }

    /**
     * Configures MK4c module's encoders, conversion factor, PID, and current limit.
     */
    private void configMotors() {

        turnMotor.setMotorInverted(MK4cSwerveModuleConstants.INVERT_TURNING_MOTOR);

        driveMotor.setGearRatio(MK4cSwerveModuleConstants.DRIVE_GEAR_RATIO);

        // Apply unit conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPS, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        driveMotor.setUnitConversionFactor(MK4cSwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);

        // Apply unit conversion factor for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        turnMotor.setUnitConversionFactor(MK4cSwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR);

        // Set status signal update frequencies, optimized for swerve
        driveMotor.setTelemetryPreference(TelemetryPreference.SWERVE);
        turnMotor.setTelemetryPreference(TelemetryPreference.SWERVE);

        // We only want to ask for the abs encoder in real life
        if (FieldConstants.IS_REAL) {
            turnMotor.setEncoder(turnEncoder.getDeviceID(), MK4cSwerveModuleConstants.TURNING_MOTOR_REDUCTION);
        } else {
            turnMotor.setGearRatio(MK4cSwerveModuleConstants.TURNING_MOTOR_REDUCTION);
        }

        turnMotor.setPositionClosedLoopWrappingEnabled(true);

        setDriveGains(MK4cSwerveModuleConstants.DRIVING_GAINS);
        setTurnGains(MK4cSwerveModuleConstants.TURNING_GAINS);

        driveMotor.setStatorCurrentLimit(MK4cSwerveModuleConstants.DRIVING_MOTOR_STATOR_LIMIT_AMPS);
        driveMotor.setTorqueCurrentLimits(-MK4cSwerveModuleConstants.DRIVING_MOTOR_TORQUE_LIMIT_AMPS, MK4cSwerveModuleConstants.DRIVING_MOTOR_TORQUE_LIMIT_AMPS);

        turnMotor.setStatorCurrentLimit(MK4cSwerveModuleConstants.TURNING_MOTOR_STATOR_LIMIT_AMPS);
        turnMotor.setTorqueCurrentLimits(-MK4cSwerveModuleConstants.TURNING_MOTOR_TORQUE_LIMIT_AMPS, MK4cSwerveModuleConstants.TURNING_MOTOR_TORQUE_LIMIT_AMPS);

        setDriveBrakeMode(true);
        setTurnBrakeMode(true);
    }

    private void configEncoder(double absoluteEncoderOffset) {
        turnEncoder.configureMagnetSensor(false, absoluteEncoderOffset);
        turnEncoder.setPositionConversionFactor(MK4cSwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
        turnEncoder.setVelocityConversionFactor(MK4cSwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);
    }

    /**
     * Updates the inputs sent to the Mk4c module.
     */
    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        // Call refreshALl() to refresh all status signals, and check in on him :)
        inputs.driverMotorConnected = driveMotor.refreshSignals().isOK();
        // 800 m is roughly the value at which the talonfx position "flips", leading to odometry bugs
        inputs.drivePositionFlipped = Math.abs(driveMotor.getPositionAsDouble() - inputs.drivePositionMeters) >= 1600d;
        inputs.drivePositionMeters = driveMotor.getPositionAsDouble();
        inputs.driveVelocityMPS = driveMotor.getVelocityAsDouble();
        inputs.driveAppliedVolts = driveMotor.getVoltageAsDouble();
        inputs.driveTorqueCurrentAmps = driveMotor.getTorqueCurrentAsDouble();
        inputs.driveTempCelcius = driveMotor.getTemperatureAsDouble();
        
        // Call refreshALl() to refresh all status signals, and check in on him :)
        inputs.turnMotorConnected = turnMotor.refreshSignals().isOK();
        inputs.turnInternalPositionRads = turnMotor.getPositionAsDouble();
        inputs.turnInternalVelocityRadsPerSec = turnMotor.getVelocityAsDouble();
        inputs.turnAppliedVolts = turnMotor.getVoltageAsDouble();
        inputs.turnTorqueCurrentAmps = driveMotor.getTorqueCurrentAsDouble();
        inputs.turnTempCelcius = turnMotor.getTemperatureAsDouble();

        inputs.turnEncoderConnected = turnEncoder.refreshSignals().isOK() && FieldConstants.IS_REAL;

        // // Call refreshALl() to refresh all status signals (only if he is real)
        if (inputs.turnEncoderConnected) {
            inputs.turnEncoderAbsPositionRads = turnEncoder.getAbsolutePositionAsDouble();
        } else {
            inputs.turnEncoderAbsPositionRads = inputs.turnInternalPositionRads;
        }

    }

    /**
     * Resets drive encoder to 0.
     */
    @Override
    public void resetDriveEncoder()  {
        driveMotor.resetEncoder(0);
    }

    @Override
    public void setDriveBrakeMode(boolean brake) {
        driveMotor.setBrakeMode(brake);
    }

    @Override
    public void setTurnBrakeMode(boolean brake) {
        turnMotor.setBrakeMode(brake);
    }

    @Override
    public void runDriveCharacterization(double input, double turnAngle) {
        turnMotor.setTargetPosition(turnAngle);
        driveMotor.setTorqueCurrentOutput(input);
    }

    @Override
    public void runTurnCharacterization(double input) {
        turnMotor.setTorqueCurrentOutput(input);
    }

    @Override
    public void runDriveVelocity(double velocity, double feedforward) {
        driveMotor.setTargetVelocity(velocity, feedforward);
    }

    @Override
    public void setTurnPosition(double position) {
        turnMotor.setTargetPosition(position);
    }
    
    @Override
    public void setTurnVelocity(double velocity) {
        turnMotor.setTargetVelocity(velocity);
    }

    @Override
    public void setDriveGains(GainConstants gains) {
        driveMotor.setGains(gains);
    }

    @Override
    public void setTurnGains(GainConstants gains) {
        turnMotor.setGains(gains);
    }

}
