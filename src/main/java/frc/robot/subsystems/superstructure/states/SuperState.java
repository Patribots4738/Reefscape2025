package frc.robot.subsystems.superstructure.states;

import java.util.function.BooleanSupplier;

public class SuperState {

    public final ArmState armState;
    public final ClimbState climbState;
    public final ClawState clawState;

    public SuperState(ArmState armState, ClimbState climbState, ClawState clawState) {
        this.armState = armState;
        this.climbState = climbState;
        this.clawState = clawState;
    }

    public class ArmState {
    
        public double elevatorPosition;
        public double wristPosition;
    
        public ArmState(double elevatorPosition, double wristPosition) {
            this.elevatorPosition = elevatorPosition;
            this.wristPosition = wristPosition;
        }
    }

    public class ClimbState {

        public double climbPosition;
    
        public ClimbState(double climbPosition) {
            this.climbPosition = climbPosition;
        }
        
    }

    public class ClawState {
    
        public double coralPercent;
        public double algaePercent;
        public final BooleanSupplier coralInterruptSupplier;
        public final BooleanSupplier algaeInterruptSupplier;

        public ClawState(double coralPercent, double algaePercent, BooleanSupplier coralInterruptSupplier, BooleanSupplier algaeInterruptSupplier) {
            this.coralPercent = algaePercent;
            this.algaePercent = algaePercent;
            this.coralInterruptSupplier = coralInterruptSupplier;
            this.algaeInterruptSupplier = algaeInterruptSupplier;
        }

        public ClawState(double coralPercent, double algaePercent) {
            this(coralPercent, algaePercent, () -> false, () -> false);
        }

    }

}
