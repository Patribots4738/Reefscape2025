package frc.robot.subsystems.superstructure.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
    
    @AutoLog
    class ClawIOInputs {

        public boolean motorConnected = false;
        public double percentOutput = 0.0;
        public double targetPercentOutput = 0.0;
        public double appliedOutputVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double temperatureCelcius = 0.0;

    }

    public default void updateInputs(ClawIOInputs inputs) {}

    public default void setBrakeMode(boolean brake) {}

    public default void setPercent(double percent) {}

}
