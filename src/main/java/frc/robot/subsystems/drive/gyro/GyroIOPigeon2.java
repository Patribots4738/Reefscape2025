package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.hardware.phoenix.Pigeon2Custom;
import frc.robot.util.hardware.phoenix.Pigeon2Custom.TelemetryPreference;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2Custom pigeon;

    public GyroIOPigeon2(int canId) {
        pigeon = new Pigeon2Custom(canId, "Drivebase");
        pigeon.setTelemetryPreference(TelemetryPreference.YAW_ONLY);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // refresh all status signals
        inputs.isConnected = pigeon.refreshSignals().isOK();
        inputs.yawRotation2d = Rotation2d.fromDegrees(pigeon.getYawDegrees());
        inputs.yawVelocityRadsPerSec = pigeon.getYawVelocityRadiansPerSec();
    }

    @Override
    public void setYaw(double degrees) {
        pigeon.setYaw(degrees);
    }

}
