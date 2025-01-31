package frc.robot.util.custom;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LoggedGainConstants {
    
    private GainConstants gainConstants;
    private final LoggedTunableNumber pValue; 
    private final LoggedTunableNumber iValue;
    private final LoggedTunableNumber dValue;
    private final LoggedTunableNumber ffValue;
    private final LoggedTunableNumber iZoneValue;
    private final LoggedTunableNumber minOutputValue;
    private final LoggedTunableNumber maxOutputValue;
    private final LoggedTunableNumber sValue;
    private final LoggedTunableNumber vValue;
    private final LoggedTunableNumber gValue;

    public LoggedGainConstants(GainConstants constants, String key){
        this.gainConstants = constants;
        String name = key + "/Gains/";
        this.pValue = new LoggedTunableNumber(name + "0-P", constants.getP());
        this.iValue = new LoggedTunableNumber(name + "1-I", constants.getI());
        this.dValue = new LoggedTunableNumber(name + "2-D", constants.getD());
        this.ffValue = new LoggedTunableNumber(name + "3-FF", constants.getFF());        
        this.iZoneValue = new LoggedTunableNumber(name + "4-IZone", constants.getIZone());
        this.minOutputValue = new LoggedTunableNumber(name + "5-MinOutput", constants.getMinOutput());
        this.maxOutputValue = new LoggedTunableNumber(name + "6-MaxOutput", constants.getMaxOutput());
        this.sValue = new LoggedTunableNumber(name + "7-S", constants.getS());
        this.vValue = new LoggedTunableNumber(name + "8-V", constants.getV());
        this.gValue = new LoggedTunableNumber(name + "9-G", constants.getG());

    }

    public Trigger onChanged() {
        return pValue.onChanged()
            .or(iValue.onChanged())
            .or(dValue.onChanged())
            .or(ffValue.onChanged())
            .or(iZoneValue.onChanged())
            .or(minOutputValue.onChanged())
            .or(maxOutputValue.onChanged())
            .or(sValue.onChanged())
            .or(vValue.onChanged())
            .or(gValue.onChanged());
    }

    public Trigger onChanged(Command command) {
        return this.onChanged().onTrue(command);
    }

    public GainConstants get(){
        this.gainConstants = new GainConstants(
            pValue.get(), 
            iValue.get(),
            dValue.get(), 
            ffValue.get(), 
            iZoneValue.get(), 
            minOutputValue.get(), 
            maxOutputValue.get(), 
            sValue.get(), 
            dValue.get(), 
            gValue.get()
        );
        return gainConstants;
    }


}
