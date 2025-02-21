package frc.robot.util.custom;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LoggedTunableNumber {

    private double previousValue;
    
    public LoggedTunableNumber(String key, double defaultValue) {
        this.previousValue = defaultValue;
    }

    public LoggedTunableNumber(String key) {
        this(key, 0.0);
    }

    public boolean ifChanged() {
        return get() != previousValue && !DriverStation.isFMSAttached();
    }

    public boolean onChanged() {
        return ifChanged();
    }

    public boolean onChanged(Command command) {
        return onChanged();
    }
    
    public void periodic() {
        // previousValue = get();
        // super.periodic();
    }

    public double get() {
        return previousValue;
    }

    public void set(double val) {
        previousValue = val;
    }
}
