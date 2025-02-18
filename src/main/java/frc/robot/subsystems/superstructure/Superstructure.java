package frc.robot.subsystems.superstructure;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.claw.algae.AlgaeClaw;
import frc.robot.subsystems.superstructure.claw.coral.CoralClaw;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.util.Constants.AlgaeClawConstants;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.CoralClawConstants;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.Constants.WristConstants;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.custom.LoggedTunableNumber;
import frc.robot.subsystems.superstructure.climb.Climb;

public class Superstructure {
    
    private final AlgaeClaw algaeClaw;
    private final CoralClaw coralClaw;
    private final Elevator elevator;
    private final Wrist wrist;
    private final Climb climb;
    
    private final Supplier<Pose2d> robotPoseSupplier;

    private final LoggedTunableNumber wristUnderTransition = new LoggedTunableNumber("Wrist/UnderTransitionPosition", WristConstants.UNDER_TRANSITION_RADIANS);
    private final LoggedTunableNumber wristReefTransition = new LoggedTunableNumber("Wrist/ReefTransitionPosition", WristConstants.REEF_TRANSITION_RADIANS);

    public final SuperState STOW;
    public final SuperState INTAKE;
    public final SuperState L1;
    public final SuperState L2;
    public final SuperState L3;
    public final SuperState L4;
    public final SuperState L1_PLACE;
    public final SuperState L2_PLACE;
    public final SuperState L3_PLACE;
    public final SuperState L4_PLACE;
    public final SuperState L2_ALGAE;
    public final SuperState L3_ALGAE;
    public final SuperState L2_ALGAE_IN;
    public final SuperState L3_ALGAE_IN;
    public final SuperState CLIMB_READY;
    public final SuperState CLIMB_FINAL;

    private SuperState targetState;
    private ArmState targetArmState;
    private ClimbState targetClimbState;

    public Superstructure(AlgaeClaw algaeClaw, CoralClaw coralClaw, Elevator elevator, Wrist wrist, Climb climb, Supplier<Pose2d> robotPoseSupplier) {
        this.algaeClaw = algaeClaw;
        this.coralClaw = coralClaw;
        this.elevator = elevator;
        this.wrist = wrist;
        this.climb = climb;

        this.robotPoseSupplier = robotPoseSupplier;

        STOW = new LoggedSuperState("STOW", ArmState.STOW, ClimbState.STOW, ClawState.STOP);

        targetState = STOW;
        targetArmState = ArmState.STOW;
        targetClimbState = ClimbState.STOW;

        INTAKE = new LoggedSuperState("INTAKE", ArmState.INTAKE, ClimbState.STOW, ClawState.CORAL_IN, () -> elevator.atPosition(targetState.armState.elevatorPosition), () -> false);
        L1 = new LoggedSuperState("L1", ArmState.L1, ClimbState.STOW, ClawState.STOP);
        L2 = new LoggedSuperState("L2", ArmState.L2, ClimbState.STOW, ClawState.STOP);
        L3 = new LoggedSuperState("L3", ArmState.L3, ClimbState.STOW, ClawState.STOP);
        L4 = new LoggedSuperState("L4", ArmState.L4, ClimbState.STOW, ClawState.STOP);
        L1_PLACE = new LoggedSuperState("L1_PLACE", ArmState.L1, ClimbState.STOW, ClawState.CORAL_OUT, this::armAtTargetPosition, () -> false);
        L2_PLACE = new LoggedSuperState("L2_PLACE", ArmState.L2, ClimbState.STOW, ClawState.CORAL_OUT, this::armAtTargetPosition, () -> false);
        L3_PLACE = new LoggedSuperState("L3_PLACE", ArmState.L3, ClimbState.STOW, ClawState.CORAL_OUT, this::armAtTargetPosition, () -> false);
        L4_PLACE = new LoggedSuperState("L4_PLACE", ArmState.L4, ClimbState.STOW, ClawState.CORAL_OUT, this::armAtTargetPosition, () -> false);
        L2_ALGAE = new LoggedSuperState("L2_ALGAE", ArmState.L2_ALGAE, ClimbState.STOW, ClawState.STOP);
        L3_ALGAE = new LoggedSuperState("L3_ALGAE", ArmState.L3_ALGAE, ClimbState.STOW, ClawState.STOP);
        L2_ALGAE_IN = new LoggedSuperState("L2_ALGAE_IN", ArmState.L2_ALGAE, ClimbState.STOW, ClawState.ALGAE_IN);
        L3_ALGAE_IN = new LoggedSuperState("L3_ALGAE_IN", ArmState.L3_ALGAE, ClimbState.STOW, ClawState.ALGAE_IN);
        CLIMB_READY = new LoggedSuperState("CLIMB_READY", ArmState.CLIMB, ClimbState.READY, ClawState.STOP);
        CLIMB_FINAL = new LoggedSuperState("CLIMB_FINAL", ArmState.CLIMB, ClimbState.FINAL, ClawState.STOP);

    }

    public enum ArmState {

        STOW (ElevatorConstants.STOW_POSITION_METERS, WristConstants.STOW_POSITION_RADIANS),
        INTAKE (ElevatorConstants.INTAKE_POSITION_METERS, WristConstants.INTAKE_POSITION_RADIANS),
        L1 (ElevatorConstants.L1_POSITION_METERS, WristConstants.L1_POSITION_RADIANS),
        L2 (ElevatorConstants.L2_POSITION_METERS, WristConstants.L2_POSITION_RADIANS),
        L3 (ElevatorConstants.L3_POSITION_METERS, WristConstants.L3_POSITION_RADIANS),
        L4 (ElevatorConstants.L4_POSITION_METERS, WristConstants.L4_POSITION_RADIANS),
        CLIMB (ElevatorConstants.STOW_POSITION_METERS, WristConstants.CLIMB_RADIANS),
        L2_ALGAE (ElevatorConstants.L2_POSITION_REMOVE_ALGAE, WristConstants.ALGAE_REMOVAL),
        L3_ALGAE (ElevatorConstants.L3_POSITION_REMOVE_ALGAE, WristConstants.ALGAE_REMOVAL);

        double elevatorPosition, wristPosition;

        ArmState(double elevatorPosition, double wristPosition) {
            this.elevatorPosition = elevatorPosition;
            this.wristPosition = wristPosition;
        }

    }

    public enum ClimbState {

        STOW (ClimbConstants.STOW_POSITION_RADIANS),
        READY (ClimbConstants.READY_POSITION_RADIANS),
        FINAL (ClimbConstants.FINAL_POSITION_RADIANS);

        double climbPosition;

        ClimbState(double climbPosition) {
            this.climbPosition = climbPosition;
        }

    }

    public enum ClawState {

        STOP (0, 0),
        CORAL_IN (CoralClawConstants.INTAKE_PERCENT, 0),
        CORAL_OUT (CoralClawConstants.OUTTAKE_PERCENT, 0),
        ALGAE_IN (0, AlgaeClawConstants.INTAKE_PERCENT),
        ALGAE_OUT (0, AlgaeClawConstants.OUTTAKE_PERCENT),
        BOTH_IN (CoralClawConstants.INTAKE_PERCENT, AlgaeClawConstants.INTAKE_PERCENT),
        BOTH_OUT (CoralClawConstants.OUTTAKE_PERCENT, AlgaeClawConstants.INTAKE_PERCENT);

        double coralPercent, algaePercent;

        ClawState(double coralPercent, double algaePercent) {
            this.coralPercent = coralPercent;
            this.algaePercent = algaePercent;
        }

    }

    // This command allows us to consistently transition between superstructure states.
    // Note that this command is not safe if the end configuration is not possible, for example if the final goal has the claw inside the climb.
    public Command setSuperState(SuperState nextState) {
        // Update logged state
        return Commands.runOnce(() -> this.targetState = nextState).alongWith(
            Commands.sequence(
                // Run these commands in parallel, cancel all when first command argument ends
                Commands.deadline(
                    fixArmAndClimb(nextState.armState, nextState.climbState),
                    // While the arm and climb are having their little dance, the claw(s) wait until its their turn to go in parallel with the rest of this command
                    Commands.sequence(
                        Commands.waitUntil(() -> nextState.coralInterruptSupplier.getAsBoolean() || nextState.clawState.coralPercent == 0),
                        coralClaw.setPercentCommand(() -> nextState.clawState.coralPercent)
                    ),
                    Commands.sequence(
                        Commands.waitUntil(() -> nextState.algaeInterruptSupplier.getAsBoolean() || nextState.clawState.algaePercent == 0),
                        algaeClaw.setPercentCommand(() -> nextState.clawState.algaePercent)
                    )
                ),
                // This is for if the suppliers never return true, then make the claws do their state thing
                Commands.parallel(
                    coralClaw.setPercentCommand(() -> nextState.clawState.coralPercent),
                    algaeClaw.setPercentCommand(() -> nextState.clawState.algaePercent)
                )
            )
        );
    }

    public Command setArmState(ArmState state) {
        return Commands.runOnce(() -> targetArmState = state).alongWith(
            Commands.sequence(
                Commands.either(
                    // Move wrist to nearest transition pose, unless the arm was previously stowed up (which is a safe spot)
                    transitionWrist(() -> state.wristPosition), 
                    // Move wrist straight to target position
                    wrist.setPositionCommand(() -> state.wristPosition), 
                    () -> 
                        // Only transition wrist if elevator needs to move in addition to other conditions
                        (shouldEvadeReef() || state.wristPosition < wristUnderTransition.get()) 
                        && !elevator.atPosition(state.elevatorPosition)
                // Stop blocking sequence when wrist is in a safe position
                ).until(this::wristSafe),
                elevator.setPositionCommand(() -> state.elevatorPosition),
                // Below here is only effectual if wrist just transitioned
                Commands.waitUntil(() -> !shouldEvadeReef()).onlyIf(() -> state.wristPosition < wristReefTransition.get()),
                wrist.setPositionCommand(() -> state.wristPosition)
            )
        );
    }

    public Command setClimbState(ClimbState state) {
        return Commands.runOnce(() -> targetClimbState = state)
                .alongWith(climb.setPositionCommand(() -> state.climbPosition));
    }

    public Command avoidClimb(ArmState armState, ClimbState climbState) {
        return Commands.parallel(
            Commands.sequence(
                Commands.waitUntil(() -> 
                    wrist.getPosition() > ArmState.CLIMB.wristPosition 
                    || wrist.atPosition(ArmState.CLIMB.wristPosition)),
                setClimbState(climbState)
            ),
            setArmState(armState)
        );
    }

    public Command fixArmAndClimb(ArmState armState, ClimbState climbState) {
        return Commands.either(
            // Climb needs to move to achieve goal state
            Commands.either(
                // Arm goal is unsafe for climb, give way to climb then move arm back in
                Commands.sequence(
                    avoidClimb(ArmState.CLIMB, climbState),
                    setArmState(armState)
                ), 
                // Arm goal is safe for climb, move to arm goal then set climb state
                avoidClimb(armState, climbState), 
                () -> armState.wristPosition < ArmState.CLIMB.wristPosition
            ), 
            // Climb doesn't need to move, set goal arm state and update climb control loop JIC
            Commands.parallel(
                setArmState(armState),
                setClimbState(climbState)
            ), 
            () -> !climb.atPosition(climbState.climbPosition)
        );
    }

    public Command transitionWrist(DoubleSupplier targetWristPosition) {
        return Commands.either(
            wrist.setPositionCommand(wristReefTransition::get),
            wrist.setPositionCommand(wristUnderTransition::get),  
            () -> shouldEvadeReef() || targetWristPosition.getAsDouble() > wristUnderTransition.get()
        );
    }

    public Command algaeL2Command(BooleanSupplier continueIntakingSupplier)  {
        return Commands.sequence(
            setSuperState(L2_ALGAE_IN),
            Commands.waitUntil(() -> !continueIntakingSupplier.getAsBoolean()),
            setSuperState(L2_ALGAE)
        );
    }

    public Command algaeL3Command(BooleanSupplier continueIntakingSupplier)  {
        return Commands.sequence(
            setSuperState(L3_ALGAE_IN),
            Commands.waitUntil(() -> !continueIntakingSupplier.getAsBoolean()),
            setSuperState(L3_ALGAE)
        );
    }

    public Command coralIntakeCommand() {
        return 
            Commands.sequence(
                setSuperState(INTAKE),
                Commands.waitUntil(coralClaw::hasPiece),
                setSuperState(L3)
            );
    }

    public Command coralAutoIntakeStartCommand(){
        return setSuperState(INTAKE);
    }

    public Command coralAutoIntakeStopCommand(){
        return setSuperState(L3);
    }

    public SuperState getPlacementState() {
        // Derive next state based on current arm target
        SuperState placementState = switch (targetState.armState) {
            case L2 -> L2_PLACE;
            case L3 -> L3_PLACE;
            case L4 -> L4_PLACE;
            default -> L1_PLACE;
        };

        return placementState;
    }

    public SuperState getStopState() {
        // Derive next state based on current arm target
        SuperState stopState = switch (targetState.armState) {
            case L2 -> L2;
            case L3 -> L3;
            case L4 -> L4;
            default -> L1;
        };

        return stopState;
    }

    public Command outtakeCommand() {
        // Figure out the correct SuperState at runtime, so defer
        return Commands.defer(() -> setSuperState(getPlacementState()), Set.of(elevator, wrist, coralClaw, algaeClaw, climb));
    }

    public Command stopOuttakeCommand() {
        // Figure out the correct SuperState at runtime, so defer
        return Commands.defer(() -> setSuperState(getStopState()), Set.of(elevator, wrist, coralClaw, algaeClaw, climb));
    }

    public Command coralPlaceCommand() {
        return Commands.sequence(
            outtakeCommand(),
            Commands.waitUntil(() -> !coralClaw.hasPiece()),
            setSuperState(STOW)
        );
    }

    public Command stopAllCommand() {
        return 
            Commands.parallel(
                wrist.setNeutralCommand(),
                elevator.setNeutralCommand(),
                climb.setNeutralCommand(),
                algaeClaw.setNeutralCommand(),
                coralClaw.setNeutralCommand()
            );
    }

    @AutoLogOutput (key = "Subsystems/Superstructure/ArmAtTargetPosition")
    public boolean armAtTargetPosition() {
        return elevator.atPosition(targetState.armState.elevatorPosition) && wrist.atPosition(targetState.armState.wristPosition);
    }

    @AutoLogOutput (key = "Subsystems/Superstructure/WristSafe")
    public boolean wristSafe() {
        return (shouldEvadeReef() && wrist.atPosition(wristReefTransition.get())) 
            || (!shouldEvadeReef() && (wrist.atPosition(wristUnderTransition.get()) || wrist.getPosition() > wristUnderTransition.get()));
    }

    @AutoLogOutput (key = "Subsystems/Superstructure/ShouldEvadeReef")
    public boolean shouldEvadeReef() {
        return PoseCalculations.nearReef(robotPoseSupplier.get());
    }

    public SuperState getTargetState() {
        return targetState;
    }

    @AutoLogOutput (key = "Subsystems/Superstructure/TargetState/Key")
    public String getTargetStateKey() {
        return targetState.key;
    }

    @AutoLogOutput (key = "Subsystems/Superstructure/TargetState/ArmState")
    public ArmState getTargetArmState() {
        return targetArmState;
    }

    @AutoLogOutput (key = "Subsystems/Superstructure/TargetState/ClimbState")
    public ClimbState getTargetClimbState() {
        return targetClimbState;
    }

    @AutoLogOutput (key = "Subsystems/Superstructure/TargetState/ClawState")
    public ClawState getTargetClawState() {
        return targetState.clawState;
    }

}
