package frc.robot.util.hardware.rev;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ThroughBoreEncoder extends DutyCycleEncoder {

    private double positionConversionFactor = 1.0;
    
    public ThroughBoreEncoder(int dioPin, double zeroedPositionRotations) {
        super(dioPin, 1.0, zeroedPositionRotations);
    }

    public void setPositionConversionFactor(double newFactor) {
        positionConversionFactor = newFactor;
    }

    public double getPosition() {
        return get() * positionConversionFactor;
    }

}
