package frc.robot.subsystems.drive.gyro;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;

public class Gyro {

    private GyroIO io;
    private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    public Gyro(GyroIO io) {
        this.io = io;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Gyro", inputs);
    }

    public void setYaw(double yaw) {
        io.setYaw(yaw);
    }

    public Rotation2d getYawRotation2D() {
        return inputs.yawRotation2d;
    }

    public double getYaw() {
        return inputs.yawRotation2d.getRadians();
    }

    public double getYawVelocity() {
        return inputs.yawVelocityRadsPerSec;
    }
}
