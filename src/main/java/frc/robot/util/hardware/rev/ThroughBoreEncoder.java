package frc.robot.util.hardware.rev;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ThroughBoreEncoder extends DutyCycleEncoder {

    private double positionOffset = 0.0;
    private double positionConversionFactor = 1.0;
    
    public ThroughBoreEncoder(int dioPin) {
        super(dioPin);
    }

    public void setPositionOffsetRotations(double offset) {
        positionOffset = offset;
    }

    public void zeroEncoder() {
        positionOffset = -get();
    }

    public void setPositionConversionFactor(double newFactor) {
        positionConversionFactor = newFactor;
    }

    public double getPosition() {
        return (get() + positionOffset) * positionConversionFactor;
    }

}
