package frc.robot.subsystems.superstructure.states;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.custom.LoggedTunableNumber;

public class LoggedSuperState extends SuperState {
    
    LoggedTunableNumber elevatorPosition;
    LoggedTunableNumber wristPosition;
    LoggedTunableNumber climbPosition;
    LoggedTunableNumber coralPercent;
    LoggedTunableNumber algaePercent;

    public LoggedSuperState(String key, ArmState armState, ClimbState climbState, ClawState clawState) {
        super(armState, climbState, clawState);
        String mainKey = "SuperStates/" + key;
        String armKey = mainKey + "ArmState/";
        String climbKey = mainKey + "ClimbState/";
        String clawKey = mainKey + "ClawState/";
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

}
