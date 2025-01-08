package frc.robot.subsystems.drive.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;


public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean isConnected = false;
        public Rotation2d yawRotation2d = new Rotation2d();
        public double pitchRads = 0.0;
        public double rollRads = 0.0;
        public double yawVelocityRadsPerSec = 0.0;
        public double pitchVelocityRadsPerSec = 0.0;
        public double rollVelocityRadsPerSec = 0.0;
    }

    default void updateInputs(GyroIOInputs inputs) {}

    default void setYaw(double degrees) {}
}
