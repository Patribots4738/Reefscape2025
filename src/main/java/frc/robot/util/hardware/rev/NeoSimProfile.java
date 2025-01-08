package frc.robot.util.hardware.rev;

import edu.wpi.first.math.system.plant.DCMotor;

class NeoSimProfile extends NeoPhysicsSim.SimProfile {
    private final Neo neo;
    private double stallTorque = 0.0;
    private double freeSpeed = 0.0;
    private DCMotor DCMotor = null;
    private double velocity = 0.0;
    private double RPM_CONVERSION = 9.5493;

    public NeoSimProfile(Neo neo, double stallTorque, double freeSpeed) {
        this.neo = neo;
        this.stallTorque = stallTorque;
        this.freeSpeed = freeSpeed;
        
        // This method of simulation doesn't work, must implement some more wpi oriented approach
        // neo.setSimFreeSpeed((float)(this.DCMotor.freeSpeedRadPerSec this.RPM_CONVERSION));
        // neo.setSimStallTorque((float)this.DCMotor.stallTorqueNewtonMeters);
    }

    public NeoSimProfile(Neo neo, DCMotor motor) {
        this.neo = neo;
        this.DCMotor = motor;

        // This method of simulation doesn't work, must implement some more wpi oriented approach
        // neo.setSimFreeSpeed((float)(this.DCMotor.freeSpeedRadPerSec this.RPM_CONVERSION));
        // neo.setSimStallTorque((float)this.DCMotor.stallTorqueNewtonMeters);
    }

    public void run() {
        double period = this.getPeriod();
        this.velocity = this.neo.getEncoder().getVelocity();
        double position = this.neo.getEncoder().getPosition();
        double posFactor = this.neo.getPositionConversionFactor();
        this.neo.getEncoder().setPosition(position + this.velocity * period / 60000.0 * posFactor);
    }
}
