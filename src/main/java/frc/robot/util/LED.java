package frc.robot.util;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private static final int stripLength = 50;
    private static final int kPort = 2;

    private final AddressableLED patri_LED;
    private final AddressableLEDBuffer patri_LEDBuffer;

    public LED() {
        patri_LED = new AddressableLED(kPort);
        patri_LEDBuffer = new AddressableLEDBuffer(stripLength);

        patri_LED.setLength(stripLength);
        patri_LED.start();

        setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
    }

    @Override
    public void periodic() {
        patri_LED.setData(patri_LEDBuffer);
    }

    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(patri_LEDBuffer));
    }

    Color patribotsGreen = new Color(0, 200, 0);
    Color patribotsGold = new Color(255, 215, 0);

    LEDPattern green = LEDPattern.solid(patribotsGreen);
    LEDPattern gold = LEDPattern.solid(patribotsGold);

    Map<Double, Color> testSteps = Map.of(1.0, patribotsGreen, 0.5, patribotsGold);

    LEDPattern PHHSColors = LEDPattern.steps(testSteps);

    public Command patriColors() {
        return run(() -> runPattern(PHHSColors));
    }
}
