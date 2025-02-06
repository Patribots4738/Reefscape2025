package frc.robot.util.custom;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LoggedGainConstants {
    
    private GainConstants gainConstants;
    private final LoggedTunableNumber pValue; 
    private final LoggedTunableNumber iValue;
    private final LoggedTunableNumber dValue;
    private final LoggedTunableNumber aValue;
    private final LoggedTunableNumber sValue;
    private final LoggedTunableNumber vValue;
    private final LoggedTunableNumber gValue;

    public LoggedGainConstants(GainConstants constants, String key){
        this.gainConstants = constants;
        String name = key + "/Gains/";
        this.pValue = new LoggedTunableNumber(name + "0-P", constants.getP());
        this.iValue = new LoggedTunableNumber(name + "1-I", constants.getI());
        this.dValue = new LoggedTunableNumber(name + "2-D", constants.getD());
        this.aValue = new LoggedTunableNumber(name + "3-A", constants.getA());
        this.sValue = new LoggedTunableNumber(name + "4-S", constants.getS());
        this.vValue = new LoggedTunableNumber(name + "5-V", constants.getV());
        this.gValue = new LoggedTunableNumber(name + "6-G", constants.getG());

    }

    public Trigger onChanged() {
        return pValue.onChanged()
            .or(iValue.onChanged())
            .or(dValue.onChanged())
            .or(aValue.onChanged())
            .or(sValue.onChanged())
            .or(vValue.onChanged())
            .or(gValue.onChanged());
    }

    public Trigger onChanged(Command command) {
        return this.onChanged().onTrue(command);
    }

    public GainConstants get(){
        return gainConstants.withGains(
            pValue.get(),
            iValue.get(),
            dValue.get(),
            aValue.get(),
            sValue.get(),
            vValue.get(),
            gValue.get()
        );
    }


}
