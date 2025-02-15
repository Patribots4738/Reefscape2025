package frc.robot.subsystems.superstructure;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.Superstructure.ArmState;
import frc.robot.subsystems.superstructure.Superstructure.ClawState;
import frc.robot.subsystems.superstructure.Superstructure.ClimbState;
import frc.robot.util.custom.LoggedTunableNumber;

public class LoggedSuperState extends SuperState {
    
    private final LoggedTunableNumber elevatorPosition;
    private final LoggedTunableNumber wristPosition;
    private final LoggedTunableNumber climbPosition;
    private final LoggedTunableNumber coralPercent;
    private final LoggedTunableNumber algaePercent;

    public LoggedSuperState(String key, ArmState armState, ClimbState climbState, ClawState clawState, BooleanSupplier coralInterruptSupplier, BooleanSupplier algaeInterruptSupplier) {
        super(key, armState, climbState, clawState, coralInterruptSupplier, algaeInterruptSupplier);
        String mainKey = "SuperStates/" + key;
        String armKey = mainKey + "/ArmState/";
        String climbKey = mainKey + "/ClimbState/";
        String clawKey = mainKey + "/ClawState/";
        elevatorPosition = new LoggedTunableNumber(armKey + "ElevatorPosition", armState.elevatorPosition);
        wristPosition = new LoggedTunableNumber(armKey + "WristPosition", armState.wristPosition);
        climbPosition = new LoggedTunableNumber(climbKey + "ClimbPosition", climbState.climbPosition);
        coralPercent = new LoggedTunableNumber(clawKey + "CoralPercent", clawState.coralPercent);
        algaePercent = new LoggedTunableNumber(clawKey + "AlgaePercent", clawState.algaePercent);

        elevatorPosition.onChanged(Commands.runOnce(() -> this.armState.elevatorPosition = elevatorPosition.get()));
        wristPosition.onChanged(Commands.runOnce(() -> this.armState.wristPosition = wristPosition.get()));
        climbPosition.onChanged(Commands.runOnce(() -> this.climbState.climbPosition = climbPosition.get()));
        coralPercent.onChanged(Commands.runOnce(() -> this.clawState.coralPercent = coralPercent.get()));
        algaePercent.onChanged(Commands.runOnce(() -> this.clawState.algaePercent = algaePercent.get()));
    }

    public LoggedSuperState(String key, ArmState armState, ClimbState climbState, ClawState clawState) {
        this(key, armState, climbState, clawState, () -> false, () -> false);
    }

}
