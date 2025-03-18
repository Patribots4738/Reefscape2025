package frc.robot.subsystems.superstructure;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.superstructure.Superstructure.ArmState;
import frc.robot.subsystems.superstructure.Superstructure.ClawState;
import frc.robot.subsystems.superstructure.Superstructure.ClimbState;

public class SuperState {

    public final String key;
    public final ArmState armState;
    public final ClimbState climbState;
    public final ClawState clawState;
    public final BooleanSupplier coralInterruptSupplier;
    public final BooleanSupplier algaeInterruptSupplier;

    public SuperState(String key, ArmState armState, ClimbState climbState, ClawState clawState, BooleanSupplier coralInterruptSupplier, BooleanSupplier algaeInterruptSupplier) {
        this.key = key;
        this.armState = armState;
        this.climbState = climbState;
        this.clawState = clawState;
        this.coralInterruptSupplier = coralInterruptSupplier;
        this.algaeInterruptSupplier = algaeInterruptSupplier;
    }

    public SuperState(String key, ArmState armState, ClimbState climbState, ClawState clawState) {
        this(key, armState, climbState, clawState, () -> false, () -> false);
    }

    public SuperState(String key, ArmState armState, ClawState clawState, BooleanSupplier coralInterruptSupplier, BooleanSupplier algaeInterruptSupplier) {
        this(key, armState, ClimbState.STOW, clawState, coralInterruptSupplier, algaeInterruptSupplier);
    }

    public SuperState(String key, ArmState armState, ClawState clawState) {
        this(key, armState, clawState, () -> false, () -> false);
    }

    public SuperState(String key, ArmState armState) {
        this(key, armState, ClawState.DEFAULT);
    }

    public SuperState(String key, ArmState armState, ClimbState climbState) {
        this(key, armState, climbState, ClawState.DEFAULT);
    }

}
