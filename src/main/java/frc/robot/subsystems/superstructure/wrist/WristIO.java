package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.custom.GainConstants;

public interface WristIO {
    
    @AutoLog
    class WristIOInputs {
        public boolean motorConnected = false;
        public double positionRads = 0.0;
        public double velocityRadsPerSec = 0.0;
        public double targetPositionRads = 0.0;
        public double appliedOutputVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
    }

    public default void updateInputs(WristIOInputs inputs) {}

    public default void setPosition(double position, double feedforward) {}

    public default void setNeutral() {}

    public default void runCharacterization(double input) {}

    public default void setBrakeMode(boolean brake) {}

    public default void setGains(GainConstants constants) {}

    public default void configureProfile(double velocity, double acceleration, double jerk) {}

}
