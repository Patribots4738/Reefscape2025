package frc.robot.commands.logging;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.custom.LoggedTunableNumber;

public class NTLoggedGainConstants {
    
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

    public NTLoggedGainConstants(GainConstants constants, String key){
        this.gainConstants = constants;
        String name = "Tunable/Constants/" + key + "/Gains/";
        this.pValue = new LoggedTunableNumber(name + "P", constants.getP());
        this.iValue = new LoggedTunableNumber(name + "I", constants.getI());
        this.dValue = new LoggedTunableNumber(name + "D", constants.getP());
        this.ffValue = new LoggedTunableNumber(name + "FF", constants.getFF());        
        this.iZoneValue = new LoggedTunableNumber(name + "IZone", constants.getIZone());
        this.minOutputValue = new LoggedTunableNumber(name + "MinOutput", constants.getMinOutput());
        this.maxOutputValue = new LoggedTunableNumber(name + "MaxOutput", constants.getMaxOutput());
        this.sValue = new LoggedTunableNumber(name + "S", constants.getS());
        this.vValue = new LoggedTunableNumber(name + "V", constants.getV());
        this.gValue = new LoggedTunableNumber(name + "G", constants.getG());

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
        .or(vValue.onChanged()).
        or(gValue.onChanged());
    }

    public Trigger onChanged(Command command) {
        return this.onChanged().onTrue(command);
    }

    public GainConstants get(){
        this.gainConstants = new GainConstants(pValue.get(), iValue.get(), dValue.get(), ffValue.get(), iZoneValue.get(), minOutputValue.get(), maxOutputValue.get(), sValue.get(), dValue.get(), gValue.get());
        return gainConstants;
    }


}
