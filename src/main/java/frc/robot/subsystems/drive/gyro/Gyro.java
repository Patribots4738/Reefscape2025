package frc.robot.subsystems.drive.gyro;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;

public class Gyro {

    private GyroIO io;
    private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    public Gyro(GyroIO io) {
        this.io = io;
        io.setYaw(0);
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Gyro", inputs);
    }

    public Rotation2d getYawRotation2D() {
        return inputs.yawRotation2d;
    }

    public double getYaw() {
        return inputs.yawRotation2d.getRadians();
    }

    public double getPitch() {
        return inputs.pitchRads;
    }

    public double getRoll() {
        return inputs.rollRads;
    }

    public double getYawVelocity() {
        return inputs.yawVelocityRadsPerSec;
    }

    public double getPitchVelocity() {
        return inputs.yawVelocityRadsPerSec;
    }

    public double getRollVelocity() {
        return inputs.yawVelocityRadsPerSec;
    }
}
