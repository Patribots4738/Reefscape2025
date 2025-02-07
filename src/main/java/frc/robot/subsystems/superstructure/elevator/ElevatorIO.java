package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.custom.GainConstants;

public interface ElevatorIO {
    
    @AutoLog
    class ElevatorIOInputs {
        public boolean leaderMotorConnected = false;
        public double leaderPositionMeters = 0.0;
        public double leaderTargetPositionMeters = 0.0;
        public double leaderVelocityMetersPerSecond = 0.0;
        public double leaderAppliedOutputVolts = 0.0;
        public double leaderSupplyCurrentAmps = 0.0;
        public double leaderStatorCurrentAmps = 0.0;
        public double leaderTorqueCurrentAmps = 0.0;
        public double leaderTemperatureCelsius = 0.0;

        public boolean followerMotorConnected = false;
        public double followerPositionMeters = 0.0;
        public double followerTargetPositionMeters = 0.0;
        public double followerVelocityMetersPerSecond = 0.0;
        public double followerAppliedOutputVolts = 0.0;
        public double followerSupplyCurrentAmps = 0.0;
        public double followerStatorCurrentAmps = 0.0;
        public double followerTorqueCurrentAmps = 0.0;
        public double followerTemperatureCelsius = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setPosition(double position) {}

    public default void setNeutral() {}

    public default void runCharacterization(double input) {}

    public default void setBrakeMode(boolean brake) {}

    public default void resetEncoders(double position) {}

    public default void setGains(GainConstants constants) {}

}
