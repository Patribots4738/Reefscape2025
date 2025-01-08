package frc.robot.subsystems.drive.module;

import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.MK4cSwerveModuleConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.CANCoderCustom;
import frc.robot.util.hardware.phoenix.Kraken;
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
        // TODO: CHANGE USETORQUECONTROL TO TRUE ONCE WE HAVE PHOENIX PRO
        driveMotor = new Kraken(drivingCANId, "Drivebase", true, false);
        turnMotor = new Kraken(turningCANId, "Drivebase", true, false);
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

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        driveMotor.setPositionConversionFactor(MK4cSwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
        driveMotor.setVelocityConversionFactor(MK4cSwerveModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        turnMotor.setPositionConversionFactor(MK4cSwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
        turnMotor.setVelocityConversionFactor(MK4cSwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

        // Set status signal update frequencies, optimized for swerve
        driveMotor.setTelemetryPreference(TelemetryPreference.SWERVE);
        turnMotor.setTelemetryPreference(TelemetryPreference.SWERVE);

        // We only want to ask for the abs encoder in real life
        if (!FieldConstants.IS_SIMULATION) {
            turnMotor.setEncoder(turnEncoder.getDeviceID(), MK4cSwerveModuleConstants.TURNING_MOTOR_REDUCTION);
        }

        turnMotor.setPositionClosedLoopWrappingEnabled(true);

        setGains(MK4cSwerveModuleConstants.DRIVING_GAINS, MK4cSwerveModuleConstants.TURNING_GAINS);

        driveMotor.setTorqueCurrentLimits(
            -MK4cSwerveModuleConstants.DRIVING_MOTOR_TORQUE_LIMIT_AMPS,
            MK4cSwerveModuleConstants.DRIVING_MOTOR_TORQUE_LIMIT_AMPS);
        turnMotor.setTorqueCurrentLimits(
            -MK4cSwerveModuleConstants.TURNING_MOTOR_TORQUE_LIMIT_AMPS,
            MK4cSwerveModuleConstants.TURNING_MOTOR_TORQUE_LIMIT_AMPS);

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
        inputs.drivePositionFlipped = (inputs.drivePositionMeters > 800 && driveMotor.getPositionAsDouble() < -800);
        inputs.drivePositionMeters = driveMotor.getPositionAsDouble();
        inputs.driveVelocityMPS = driveMotor.getVelocityAsDouble();
        inputs.driveAppliedVolts = driveMotor.getVoltageAsDouble();
        inputs.driveSupplyCurrentAmps = driveMotor.getSupplyCurrentAsDouble();
        inputs.driveStatorCurrentAmps = driveMotor.getStatorCurrentAsDouble();
        inputs.driveTempCelcius = driveMotor.getTemperatureAsDouble();
        
        // Call refreshALl() to refresh all status signals, and check in on him :)
        inputs.turnMotorConnected = turnMotor.refreshSignals().isOK();
        inputs.turnInternalPositionRads = turnMotor.getPositionAsDouble();
        inputs.turnInternalVelocityRadsPerSec = turnMotor.getVelocityAsDouble();
        inputs.turnAppliedVolts = turnMotor.getVoltageAsDouble();
        inputs.turnSupplyCurrentAmps = turnMotor.getSupplyCurrentAsDouble();
        inputs.turnStatorCurrentAmps = turnMotor.getStatorCurrentAsDouble();
        inputs.turnTempCelcius = turnMotor.getTemperatureAsDouble();

        // Call refreshALl() to refresh all status signals (only if he is real)
        if (!FieldConstants.IS_SIMULATION) {
            inputs.turnEncoderConnected = turnEncoder.refreshSignals().isOK();
        }

        inputs.turnEncoderAbsPositionRads = inputs.turnEncoderConnected ? turnEncoder.getAbsolutePositionAsDouble() : inputs.turnInternalPositionRads;
        inputs.turnEncoderPositionRads = inputs.turnEncoderConnected ? turnEncoder.getPositionAsDouble() : inputs.turnInternalPositionRads;
        inputs.turnEncoderVelocityRadsPerSec = inputs.turnEncoderConnected ? turnEncoder.getVelocityAsDouble() : inputs.turnInternalVelocityRadsPerSec;

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
        driveMotor.setVoltageOutput(input);
    }

    @Override
    public void runTurnCharacterization(double input) {
        turnMotor.setVoltageOutput(input);
    }

    @Override
    public void runDriveVelocity(double velocity) {
        driveMotor.setTargetVelocity(velocity);
    }

    @Override
    public void setTurnPosition(double position) {
        turnMotor.setTargetPosition(position);
    }

    @Override
    public void setGains(GainConstants driveGains, GainConstants turnGains) {
        driveMotor.setGains(driveGains);
        turnMotor.setGains(turnGains);
    }
    
}
