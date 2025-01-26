package frc.robot.subsystems.superstructure.climb;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.custom.GainConstants;

public interface ClimbIO {
    
    @AutoLog
    class ClimbIOInputs {
        public boolean leaderMotorConnected = false;
        public double leaderPositionRads = 0.0;
        public double leaderVelocityRadsPerSec = 0.0;
        public double leaderTargetPositionRads = 0.0;
        public double leaderAppliedOutputVolts = 0.0;
        public double leaderSupplyCurrentAmps = 0.0;
        public double leaderStatorCurrentAmps = 0.0;
        public double leaderTorqueCurrentAmps = 0.0;
        public double leaderTemperatureCelcius = 0.0;

        public boolean followerMotorConnected = false;
        public double followerPositionRads = 0.0;
        public double followerVelocityRadsPerSec = 0.0;
        public double followerTargetPositionRads = 0.0;
        public double followerAppliedOutputVolts = 0.0;
        public double followerSupplyCurrentAmps = 0.0;
        public double followerStatorCurrentAmps = 0.0;
        public double followerTorqueCurrentAmps = 0.0;
        public double followerTemperatureCelcius = 0.0;
    }

    public default void updateInputs(ClimbIOInputs inputs) {}

    public default void setPosition(double position) {}

    public default void runCharacterization(double input) {}

    public default void setBrakeMode(boolean brake) {}

    public default void setGains(GainConstants constants) {}


}
