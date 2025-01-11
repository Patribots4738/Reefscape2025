package frc.robot.util.custom;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LoggedTunableBoolean extends LoggedNetworkBoolean {

    private boolean previousValue;
    
    public LoggedTunableBoolean(String key, boolean defaultValue) {
        super("Tunable/Constants/" + key, defaultValue);
        this.previousValue = defaultValue;
    }

    public LoggedTunableBoolean(String key) {
        this(key, false);
    }

    public boolean ifChanged() {
        return get() != previousValue;
    }

    public Trigger onChanged() {
        return new Trigger(this::ifChanged);
    }

    public Trigger onChanged(Command command) {
        return this.onChanged().onTrue(command);
    }
    
    @Override
    public void periodic() {
        previousValue = get();
        super.periodic();
    }
}
