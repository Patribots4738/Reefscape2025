package frc.robot.util.custom;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LoggedTunableNumber extends LoggedNetworkNumber {

    private double previousValue;
    
    public LoggedTunableNumber(String key, double defaultValue) {
        super("Constants/" + key, defaultValue);
        this.previousValue = defaultValue;
    }

    public LoggedTunableNumber(String key) {
        this(key, 0.0);
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
