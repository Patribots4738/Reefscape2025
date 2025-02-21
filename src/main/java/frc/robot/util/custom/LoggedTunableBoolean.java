package frc.robot.util.custom;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LoggedTunableBoolean {

    private boolean previousValue;
    
    public LoggedTunableBoolean(String key, boolean defaultValue) {
        this.previousValue = defaultValue;
    }

    public LoggedTunableBoolean(String key) {
        this(key, false);
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

    public boolean get() {
        return previousValue;
    }
}
