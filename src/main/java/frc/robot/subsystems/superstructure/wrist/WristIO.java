package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    
    @AutoLog
    class WristIOInputs {
        public boolean motorConnected = false;
        public double internalPositionRads = 0.0;
        public double internalVelocityRadsPerSec = 0.0;
        public double targetPositionRads = 0.0;
        public double appliedOutputVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double temperatureCelcius = 0.0;

        public boolean encoderConnected = false;
        public double encoderPositionRads = 0.0;
    }

    public default void updateInputs(WristIOInputs inputs) {}

    public default void setPosition(double position) {}

    public default void runCharacterization(double input) {}

    public default void setBrakeMode(boolean brake) {}

}
