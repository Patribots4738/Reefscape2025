// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import frc.robot.util.Constants.CANdleConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private final CANdle led;
    private static final int LED_COUNT = 100;
    private AnimationState currentState = AnimationState.DefaultColorFlow;

    public enum AnimationState {
        DefaultColorFlow (new ColorFlowAnimation(0, 128, 0, 0, 1, LED_COUNT, Direction.Forward, 0), new ColorFlowAnimation(244, 182, 14, 0, 1, LED_COUNT, Direction.Forward, LED_COUNT / 2)),
        AutoRainbow (new RainbowAnimation(1.0, 1.0, LED_COUNT, false, 0)),
        CoralStrobe (new StrobeAnimation(255, 255, 255, 255, 1.0, LED_COUNT)),
        AlgaeStrobe (new StrobeAnimation(56, 204, 186, 0, 1.0, LED_COUNT));

        Animation[] animations;
        int r, b, g;

        AnimationState(Animation... animations) {
            this.animations = animations;
            this.r = 0;
            this.g = 0;
            this.b = 0;
        }
    }

    /** Creates a new LED. */
    public LED() {
        led = new CANdle(CANdleConstants.CANDLE_CAN_ID, "rio");
        CANdleConfiguration config = new CANdleConfiguration();
        config.statusLedOffWhenActive = false;
        config.disableWhenLOS = false;
        config.stripType = LEDStripType.GRB;
        config.brightnessScalar = 1.0;
        config.vBatOutputMode = VBatOutputMode.Modulated;
        led.configAllSettings(config, 500);

    }

    public void setAnimationState(AnimationState state) {
        this.currentState = state;
    }

    public Command setAnimationStateCommand(AnimationState state) {
        return runOnce(() -> setAnimationState(state));
    }

    public Command setAnimationStateCommand(AnimationState state, DoubleSupplier timeSeconds) {
        return Commands.sequence(
            setAnimationStateCommand(state),
            Commands.waitSeconds(timeSeconds.getAsDouble())
        ).finallyDo(() -> {
            if (currentState == state) {
                currentState = AnimationState.DefaultColorFlow;
            }
        });
    }

    @Override
    public void periodic() {
        for (int i = 0; i < currentState.animations.length; i++) {
            led.animate(currentState.animations[i], i);
        }
    }
}
