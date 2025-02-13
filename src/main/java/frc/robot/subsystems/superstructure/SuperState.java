package frc.robot.subsystems.superstructure;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.superstructure.Superstructure.ArmState;
import frc.robot.subsystems.superstructure.Superstructure.ClawState;
import frc.robot.subsystems.superstructure.Superstructure.ClimbState;

public class SuperState {

    public final ArmState armState;
    public final ClimbState climbState;
    public final ClawState clawState;
    public final BooleanSupplier coralInterruptSupplier;
    public final BooleanSupplier algaeInterruptSupplier;

    public SuperState(ArmState armState, ClimbState climbState, ClawState clawState, BooleanSupplier coralInterruptSupplier, BooleanSupplier algaeInterruptSupplier) {
        this.armState = armState;
        this.climbState = climbState;
        this.clawState = clawState;
        this.coralInterruptSupplier = coralInterruptSupplier;
        this.algaeInterruptSupplier = algaeInterruptSupplier;
    }

    public SuperState(ArmState armState, ClimbState climbState, ClawState clawState) {
        this(armState, climbState, clawState, () -> false, () -> false);
    }

}
