package frc.robot.subsystems.superstructure;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
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
import frc.robot.util.custom.ActiveConditionalCommand;
import frc.robot.subsystems.superstructure.climb.Climb;

public class Superstructure {
    
    private final AlgaeClaw algaeClaw;
    private final CoralClaw coralClaw;
    private final Elevator elevator;
    private final Wrist wrist;
    private final Climb climb;

    private final Set<Subsystem> requirements;
    
    private final Supplier<Pose2d> robotPoseSupplier;

    // private final LoggedTunableNumber wristUnderTransition = new LoggedTunableNumber("Wrist/UnderTransitionPosition", WristConstants.UNDER_TRANSITION_RADIANS);
    // private final LoggedTunableNumber wristReefTransition = new LoggedTunableNumber("Wrist/ReefTransitionPosition", WristConstants.REEF_TRANSITION_RADIANS);

    public final SuperState STOW;
    public final SuperState READY_STOW;
    public final SuperState INTAKE;
    public final SuperState CORAL_DUMP;

    public final SuperState L1;
    public final SuperState L2;
    public final SuperState L3;
    public final SuperState L4;

    public final SuperState L2_WITH_ALGAE;
    public final SuperState L3_WITH_ALGAE;

    public final SuperState L1_PREP;
    public final SuperState L2_PREP;
    public final SuperState L3_PREP;
    public final SuperState L4_PREP;

    public final SuperState L1_PLACE;
    public final SuperState L2_PLACE;
    public final SuperState L3_PLACE;
    public final SuperState L4_PLACE;

    public final SuperState L2_WITH_ALGAE_PLACE;
    public final SuperState L3_WITH_ALGAE_PLACE;

    public final SuperState L1_CONFIRM;

    public final SuperState L1_EXIT;
    public final SuperState L2_EXIT;
    public final SuperState L3_EXIT;
    public final SuperState L4_EXIT;

    public final SuperState L2_WITH_ALGAE_EXIT;
    public final SuperState L3_WITH_ALGAE_EXIT;

    public final SuperState L2_ALGAE_EXIT;
    public final SuperState L3_ALGAE_EXIT;
    public final SuperState L2_ALGAE_IN;
    public final SuperState L3_ALGAE_IN;
    public final SuperState TREE_ALGAE_IN;
    public final SuperState ALGAE_CARRY;

    public final SuperState PROCESSOR_PREP;
    public final SuperState PROCESSOR_PLACE;
    public final SuperState PROCESSOR_EXIT;

    public final SuperState CLIMB_READY;
    public final SuperState CLIMB_FINAL;

    public final SuperState NET_PREP;
    public final SuperState NET_PLACE;
    public final SuperState NET_PREP_FLICK;
    public final SuperState NET_PLACE_FLICK;
    public final SuperState NET_EXIT;

    public final SuperState PREP_ALGAE_TOSS;
    public final SuperState BACK_ALGAE_TOSS;
    public final SuperState FRONT_ALGAE_TOSS;
    public final SuperState REEF_ALGAE_TOSS;

    private SuperState targetState;
    private ArmState targetArmState;
    private ClimbState targetClimbState;

    private SuperState currentPrepState;

    public Superstructure(AlgaeClaw algaeClaw, CoralClaw coralClaw, Elevator elevator, Wrist wrist, Climb climb, Supplier<Pose2d> robotPoseSupplier) {
        this.algaeClaw = algaeClaw;
        this.coralClaw = coralClaw;
        this.elevator = elevator;
        this.wrist = wrist;
        this.climb = climb;

        this.requirements = Set.of(algaeClaw, coralClaw, elevator, wrist, climb);

        this.robotPoseSupplier = robotPoseSupplier;

        STOW = new SuperState("STOW", ArmState.STOW);

        targetState = STOW;
        targetArmState = ArmState.STOW;
        targetClimbState = ClimbState.STOW;

        READY_STOW = new SuperState("READY_STOW", ArmState.INTAKE);
        INTAKE = new SuperState("INTAKE", ArmState.INTAKE, ClawState.CORAL_IN, () -> elevator.atPosition(targetState.armState.elevatorPosition), () -> false);
        CORAL_DUMP = new SuperState("CORAL_DUMP", ArmState.CORAL_DUMP, ClawState.CORAL_OUT);

        L1_PREP = new SuperState("L1_PREP", ArmState.L1_PREP);
        L2_PREP = new SuperState("L2_PREP", ArmState.L2_PREP);
        L3_PREP = new SuperState("L3_PREP", ArmState.L3_PREP);
        L4_PREP = new SuperState("L4_PREP", ArmState.L4_PREP);

        currentPrepState = L4_PREP;

        L1 = new SuperState("L1", ArmState.L1);
        L2 = new SuperState("L2", ArmState.L2);
        L3 = new SuperState("L3", ArmState.L3);
        L4 = new SuperState("L4", ArmState.L4);

        L2_WITH_ALGAE = new SuperState("L2_WITH_ALGAE", ArmState.L2_WITH_ALGAE);
        L3_WITH_ALGAE = new SuperState("L3_WITH_ALGAE", ArmState.L3_WITH_ALGAE);

        L1_PLACE = new SuperState("L1_PLACE", ArmState.L1, ClawState.L1_CORAL_OUT, () -> wrist.getPosition() > ArmState.L1.wristPosition || wrist.atPosition(ArmState.L1.wristPosition), () -> false);
        L2_PLACE = new SuperState("L2_PLACE", ArmState.L2, ClawState.CORAL_OUT);
        L3_PLACE = new SuperState("L3_PLACE", ArmState.L3, ClawState.CORAL_OUT);
        L4_PLACE = new SuperState("L4_PLACE", ArmState.L4, ClawState.CORAL_OUT);

        L2_WITH_ALGAE_PLACE = new SuperState("L2_WITH_ALGAE_PLACE", ArmState.L2_WITH_ALGAE, ClawState.CORAL_OUT);
        L3_WITH_ALGAE_PLACE = new SuperState("L3_WITH_ALGAE_PLACE", ArmState.L3_WITH_ALGAE, ClawState.CORAL_OUT);

        L1_CONFIRM = new SuperState("L1_CONFIRM", ArmState.L1_PLACE, ClawState.L1_CORAL_OUT, () -> true, () -> false);
    
        L1_EXIT = new SuperState("L1_EXIT", ArmState.L1_EXIT, ClawState.CORAL_OUT);
        L2_EXIT = new SuperState("L2_EXIT", ArmState.L2_EXIT, ClawState.CORAL_OUT);
        L3_EXIT = new SuperState("L3_EXIT", ArmState.L3_EXIT, ClawState.CORAL_OUT);
        L4_EXIT = new SuperState("L4_EXIT", ArmState.L4_EXIT, ClawState.CORAL_OUT);

        L2_WITH_ALGAE_EXIT = new SuperState("L2_WITH_ALGAE_EXIT", ArmState.L2_WITH_ALGAE_EXIT, ClawState.CORAL_OUT);
        L3_WITH_ALGAE_EXIT = new SuperState("L3_WITH_ALGAE_EXIT", ArmState.L3_WITH_ALGAE_EXIT, ClawState.CORAL_OUT);

        L2_ALGAE_EXIT = new SuperState("L2_ALGAE_EXIT", ArmState.L2_ALGAE_EXIT, ClawState.ALGAE_IN);
        L3_ALGAE_EXIT = new SuperState("L3_ALGAE_EXIT", ArmState.L3_ALGAE_EXIT, ClawState.ALGAE_IN);
        L2_ALGAE_IN = new SuperState("L2_ALGAE_IN", ArmState.L2_ALGAE, ClawState.ALGAE_IN);
        L3_ALGAE_IN = new SuperState("L3_ALGAE_IN", ArmState.L3_ALGAE, ClawState.ALGAE_IN);
        TREE_ALGAE_IN = new SuperState("TREE_ALGAE_IN", ArmState.GROUND_ALGAE_IN, ClawState.ALGAE_IN);
        ALGAE_CARRY = new SuperState("ALGAE_CARRY", ArmState.ALGAE_CARRY, ClawState.ALGAE_IN, () -> false, () -> true);

        PROCESSOR_PREP = new SuperState("PROCESSOR_PREP", ArmState.PROCESSOR, ClawState.DEFAULT);
        PROCESSOR_PLACE = new SuperState("PROCESSOR_PLACE", ArmState.PROCESSOR, ClawState.PROCESSOR_ALGAE_OUT);
        PROCESSOR_EXIT = new SuperState("PROCESSOR_EXIT", ArmState.PROCESSOR, ClawState.DEFAULT);

        CLIMB_READY = new SuperState("CLIMB_READY", ArmState.CLIMB, ClimbState.READY);
        CLIMB_FINAL = new SuperState("CLIMB_FINAL", ArmState.CLIMB, ClimbState.FINAL);

        NET_PREP = new SuperState("NET_PREP", ArmState.NET_PREP);
        NET_PLACE = new SuperState("NET_PLACE", ArmState.NET, ClawState.ALGAE_OUT, () -> false, () -> wrist.getPosition() > 1.6);
        NET_PREP_FLICK = new SuperState("NET_PREP_FLICK", ArmState.NET_PREP_FLICK);
        NET_PLACE_FLICK = new SuperState("NET_PLACE_FLICK", ArmState.NET, ClawState.ALGAE_OUT, () -> false, () -> true);
        NET_EXIT = new SuperState("NET_EXIT", ArmState.NET_EXIT);

        PREP_ALGAE_TOSS = new SuperState("PREP_ALGAE_TOSS", ArmState.PREP_ALGAE_TOSS, ClawState.ALGAE_IN, () -> false, () -> true);
        BACK_ALGAE_TOSS = new SuperState("BACK_ALGAE_TOSS", ArmState.BACK_ALGAE_TOSS, ClawState.ALGAE_OUT);
        FRONT_ALGAE_TOSS = new SuperState("FRONT_ALGAE_TOSS", ArmState.FRONT_ALGAE_TOSS, ClawState.ALGAE_OUT);
        REEF_ALGAE_TOSS = new SuperState("REEF_ALGAE_TOSS", ArmState.REEF_ALGAE_TOSS, ClawState.ALGAE_OUT);

    }

    public enum ArmState {

        STOW (ElevatorConstants.STOW_POSITION_METERS, WristConstants.STOW_POSITION_RADIANS),
        INTAKE (ElevatorConstants.INTAKE_POSITION_METERS, WristConstants.INTAKE_POSITION_RADIANS),
        CORAL_DUMP(ElevatorConstants.STOW_POSITION_METERS, WristConstants.DUMP_POSITION_RADIANS),
        L1_PREP (ElevatorConstants.L1_POSITION_METERS, WristConstants.TRANSITION_RADIANS),
        L2_PREP (ElevatorConstants.L2_POSITION_METERS, WristConstants.TRANSITION_RADIANS),
        L3_PREP (ElevatorConstants.L3_POSITION_METERS, WristConstants.TRANSITION_RADIANS),
        L4_PREP (ElevatorConstants.L3_POSITION_METERS, WristConstants.TRANSITION_RADIANS),
        L1 (ElevatorConstants.L1_POSITION_METERS, WristConstants.L1_POSITION_RADIANS),
        L2 (ElevatorConstants.L2_POSITION_METERS, WristConstants.L2_POSITION_RADIANS),
        L3 (ElevatorConstants.L3_POSITION_METERS, WristConstants.L3_POSITION_RADIANS),
        L4 (ElevatorConstants.L4_POSITION_METERS, WristConstants.L4_POSITION_RADIANS),
        L2_WITH_ALGAE (ElevatorConstants.L2_WITH_ALGAE_METERS, WristConstants.L2_WITH_ALGAE_RADIANS),
        L3_WITH_ALGAE (ElevatorConstants.L3_WITH_ALGAE_METERS, WristConstants.L3_WITH_ALGAE_RADIANS),
        L2_WITH_ALGAE_EXIT (ElevatorConstants.L2_WITH_ALGAE_METERS, WristConstants.L2_WITH_ALGAE_RADIANS),
        L3_WITH_ALGAE_EXIT (ElevatorConstants.L3_WITH_ALGAE_METERS, WristConstants.L3_WITH_ALGAE_RADIANS),
        L1_PLACE (ElevatorConstants.L1_POSITION_METERS, WristConstants.L1_PLACE_POSITION_RADIANS),
        L1_EXIT (ElevatorConstants.L1_POSITION_METERS, WristConstants.L1_PLACE_POSITION_RADIANS),
        L2_EXIT (ElevatorConstants.L2_POSITION_METERS, WristConstants.L2_POSITION_RADIANS),
        L3_EXIT (ElevatorConstants.L3_POSITION_METERS, WristConstants.L3_POSITION_RADIANS),
        L4_EXIT (ElevatorConstants.L4_POSITION_METERS, WristConstants.MAX_ANGLE_RADIANS),
        CLIMB (ElevatorConstants.STOW_POSITION_METERS, WristConstants.CLIMB_RADIANS),
        L2_ALGAE (ElevatorConstants.L2_POSITION_REMOVE_ALGAE, WristConstants.L2_ALGAE_REMOVAL),
        L3_ALGAE (ElevatorConstants.L3_POSITION_REMOVE_ALGAE, WristConstants.L3_ALGAE_REMOVAL),
        L2_ALGAE_EXIT (ElevatorConstants.L2_POSITION_REMOVE_ALGAE, WristConstants.L3_POSITION_RADIANS - 0.2),
        L3_ALGAE_EXIT (ElevatorConstants.L3_POSITION_REMOVE_ALGAE, WristConstants.L3_POSITION_RADIANS),
        ALGAE_CARRY(ElevatorConstants.L2_POSITION_REMOVE_ALGAE, WristConstants.L3_POSITION_RADIANS - 0.2),
        PROCESSOR (ElevatorConstants.PROCESSOR_METERS, WristConstants.PROCESSOR_RADIANS),
        GROUND_ALGAE_IN (ElevatorConstants.PROCESSOR_METERS, WristConstants.PROCESSOR_RADIANS),
        NET_PREP (ElevatorConstants.NET_PREP_METERS, WristConstants.NET_PREP_RADIANS),
        NET_PREP_FLICK (ElevatorConstants.NET_PLACE_METERS, WristConstants.L3_ALGAE_REMOVAL),
        NET (ElevatorConstants.NET_PLACE_METERS, WristConstants.NET_RADIANS),
        NET_EXIT (ElevatorConstants.NET_PLACE_METERS, WristConstants.NET_RADIANS),
        PREP_ALGAE_TOSS ( ElevatorConstants.L2_POSITION_REMOVE_ALGAE, WristConstants.L3_ALGAE_REMOVAL),
        BACK_ALGAE_TOSS (ElevatorConstants.L2_POSITION_REMOVE_ALGAE, WristConstants.BACK_ALGAE_TOSS),
        FRONT_ALGAE_TOSS (ElevatorConstants.L2_POSITION_REMOVE_ALGAE, WristConstants.FRONT_ALGAE_TOSS),
        REEF_ALGAE_TOSS (ElevatorConstants.L3_POSITION_METERS, WristConstants.MAX_ANGLE_RADIANS);

        public double elevatorPosition, wristPosition;

        ArmState(double elevatorPosition, double wristPosition) {
            this.elevatorPosition = elevatorPosition;
            this.wristPosition = wristPosition;
        }

    }

    public enum ClimbState {

        STOW (ClimbConstants.STOW_POSITION_RADIANS, false),
        READY (ClimbConstants.READY_POSITION_RADIANS, false),
        FINAL (ClimbConstants.FINAL_POSITION_RADIANS, true);

        public double climbPosition;
        public boolean slam;

        ClimbState(double climbPosition, boolean slam) {
            this.climbPosition = climbPosition;
            this.slam = slam;
        }

    }

    public enum ClawState {

        DEFAULT (0, 0),
        CORAL_IN (CoralClawConstants.INTAKE_PERCENT, 0),
        CORAL_OUT (CoralClawConstants.OUTTAKE_PERCENT, 0),
        L1_CORAL_OUT (CoralClawConstants.L1_OUTTAKE_PERCENT, 0),
        ALGAE_IN (0, AlgaeClawConstants.INTAKE_PERCENT),
        ALGAE_OUT (0, AlgaeClawConstants.OUTTAKE_PERCENT),
        PROCESSOR_ALGAE_OUT (0, AlgaeClawConstants.PROCESSOR_OUTTAKE_PERCENT),
        BOTH_IN (CoralClawConstants.INTAKE_PERCENT, AlgaeClawConstants.INTAKE_PERCENT),
        BOTH_OUT (CoralClawConstants.OUTTAKE_PERCENT, AlgaeClawConstants.INTAKE_PERCENT);

        public double coralPercent, algaePercent;

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
            // Run these commands in parallel
            Commands.parallel(
                fixArmAndClimb(nextState.armState, nextState.climbState),
                // While the arm and climb are having their little dance, the claw(s) wait until its their turn to go in parallel with the rest of this command
                setCoralClawState(nextState),
                setAlgaeClawState(nextState)
            )
        );
    }

    public Command setArmState(ArmState state) {
        return Commands.runOnce(() -> targetArmState = state).alongWith(
            Commands.sequence(
                Commands.race(
                    Commands.either(
                        // Move wrist to nearest transition pose, unless the arm was previously stowed up (which is a safe spot)
                        transitionWrist(() -> state.wristPosition), 
                        // Move wrist straight to target position
                        wrist.setPositionCommand(() -> state.wristPosition, this::shouldRunWristFast), 
                        () -> 
                            // Only transition wrist if elevator needs to move in addition to other conditions
                            state.wristPosition < WristConstants.UNDER_THRESHOLD_RADIANS 
                                && (!elevator.atPosition(state.elevatorPosition) || !elevator.atTargetPosition())
                    ),
                    Commands.sequence(
                        Commands.waitSeconds(0.05),
                        Commands.waitUntil(this::wristSafe)
                    )
                ),
                elevator.setPositionCommand(() -> state.elevatorPosition, this::shouldRunElevatorFast),
                // Below here is only effectual if wrist just transitioned
                wrist.setPositionCommand(() -> state.wristPosition, this::shouldRunWristFast)
            )
        );
    }

    public Command setClimbState(ClimbState state) {
        return Commands.runOnce(() -> targetClimbState = state)
                .alongWith(climb.setPositionCommand(() -> state.climbPosition, () -> state.slam));
    }

    public Command setCoralClawState(SuperState state) {
        return Commands.sequence(
            Commands.waitUntil(() -> state.coralInterruptSupplier.getAsBoolean() || state.clawState.coralPercent == 0 || structureAtTargetPosition()),
            Commands.either(
                coralClaw.setPercentCommand(() -> state.clawState.coralPercent),
                Commands.either(
                    coralClaw.setPercentCommand(CoralClawConstants.HOLD_PERCENT), 
                    coralClaw.setPercentCommand(0), 
                    coralClaw::hasPiece
                ), 
                () -> state.clawState.coralPercent != 0
            )
        );
    }

    public Command setAlgaeClawState(SuperState state) {
        return Commands.sequence(
            Commands.waitUntil(() -> state.algaeInterruptSupplier.getAsBoolean() || state.clawState.algaePercent == 0 || structureAtTargetPosition()),
            Commands.either(
                algaeClaw.setPercentCommand(() -> state.clawState.algaePercent),
                Commands.either(
                    algaeClaw.setPercentCommand(AlgaeClawConstants.HOLD_PERCENT), 
                    algaeClaw.setPercentCommand(0), 
                    algaeClaw::hasPiece
                ), 
                () -> state.clawState.algaePercent != 0
            )
        );
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
            // Climb needs to move to achieve goal state()
            Commands.sequence(
                setArmState(ArmState.CLIMB),
                setClimbState(climbState),
                setArmState(armState)
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
        return wrist.setPositionCommand(WristConstants.TRANSITION_RADIANS, this::shouldRunWristFast);
    }

    public Command algaeTreeCommand() {
        return setSuperState(TREE_ALGAE_IN);
    }
    
    public Command algaeRemovalCommand() {
        return Commands.sequence(
            new ActiveConditionalCommand(
                setSuperState(L3_ALGAE_IN),
                setSuperState(L2_ALGAE_IN),
                () -> PoseCalculations.isHighAlgaeReefSide(robotPoseSupplier.get())
            ).repeatedly().until(algaeClaw::hasPiece),
            setSuperState(L3_WITH_ALGAE)
        );
    }

    public Command algaeRemovalAutoStartCommand() {
        return new ActiveConditionalCommand(
            setSuperState(L3_ALGAE_IN),
            setSuperState(L2_ALGAE_IN),
            () -> PoseCalculations.isHighAlgaeReefSide(robotPoseSupplier.get())
        ).repeatedly().until(algaeClaw::hasPiece);
    }

    public Command algaeRemovalAutoStopCommand() {
        return Commands.either(
            setSuperState(L3_ALGAE_EXIT), 
            setSuperState(L2_ALGAE_EXIT), 
            () -> PoseCalculations.isHighAlgaeReefSide(robotPoseSupplier.get())
        );
    }

    public Command coralIntakeCommand(BooleanSupplier continueIntakingSupplier) {
        return 
            Commands.sequence(
                setSuperState(INTAKE),
                Commands.waitUntil(() -> !continueIntakingSupplier.getAsBoolean() && coralClaw.hasPiece()),
                setSuperState(READY_STOW)
            );
    }

    public Command coralPrepCommand() {
        return Commands.defer(() -> setSuperState(currentPrepState), requirements);
    }

    public Command coralAutoIntakeStartCommand(){
        return setSuperState(INTAKE);
    }

    public Command coralAutoIntakeStopCommand(){
        return setSuperState(READY_STOW);
    }

    public SuperState getPlacementState() {
        // Derive next state based on current arm target
        SuperState placementState = switch (targetState.armState) {
            case L1 -> L1_PLACE;
            case L2 -> L2_PLACE;
            case L2_WITH_ALGAE -> L2_WITH_ALGAE_PLACE;
            case L3, REEF_ALGAE_TOSS -> L3_PLACE;
            case L3_WITH_ALGAE -> L3_WITH_ALGAE_PLACE;
            case L4 -> L4_PLACE;
            case NET_PREP -> NET_PLACE;
            case PROCESSOR -> PROCESSOR_PLACE;
            default -> CORAL_DUMP;
        };

        if (placementState != NET_PLACE && placementState != PROCESSOR_PLACE) {
            currentPrepState = getPrepState(placementState);
        }

        return placementState;
    }

    public SuperState getPrepState(SuperState placementState) {
        SuperState prepState = switch (placementState.armState) {
            case L2 -> L2_PREP;
            case L3 -> L3_PREP;
            case L4 -> L4_PREP;
            default -> L1_PREP;
        };

        return prepState;
    }

    public SuperState getExitState() {
        // Derive next state based on current arm target
        SuperState exitState = switch (targetState.armState) {
            case L1 -> L1_EXIT;
            case L1_PLACE -> L1_EXIT;
            case L2 -> L2_EXIT;
            case L3 -> L3_EXIT;
            case L4 -> L4_EXIT;
            case L2_WITH_ALGAE -> L2_WITH_ALGAE_EXIT;
            case L3_WITH_ALGAE -> L3_WITH_ALGAE_EXIT;
            case NET -> NET_EXIT;
            case PROCESSOR -> PROCESSOR_EXIT;
            default -> READY_STOW;
        };

        return exitState;
    }

    public Command outtakeCommand() {
        // Figure out the correct SuperState at runtime, so defer
        return Commands.defer(() -> setSuperState(getPlacementState()), requirements);
    }

    public Command stopOuttakeCommand() {
        // Figure out the correct SuperState at runtime, so defer
        return Commands.defer(() -> setSuperState(getExitState()), requirements);
    }

    public Command placeCommand(BooleanSupplier continueOuttakingSupplier) {
        return Commands.sequence(
            outtakeCommand(),
            Commands.sequence(
                Commands.waitSeconds(0.15),
                setSuperState(L1_CONFIRM)
            ).onlyIf(() -> targetArmState == ArmState.L1),
            Commands.waitUntil(() -> 
                !continueOuttakingSupplier.getAsBoolean() && 
                (targetState.clawState.coralPercent != 0 && !coralClaw.hasPiece() 
                    || targetState.clawState.algaePercent != 0 && !algaeClaw.hasPiece())),
            stopOuttakeCommand(),
            Commands.waitUntil(() -> !shouldEvadeReef()),
            Commands.either(
                setSuperState(ALGAE_CARRY), 
                setSuperState(READY_STOW), 
                algaeClaw::hasPiece
            )
        );
    }

    public Command tossAlgaeCommand() {
        return Commands.either(
            setSuperState(REEF_ALGAE_TOSS).andThen(setSuperState(L3)),
            Commands.sequence(
                setSuperState(PREP_ALGAE_TOSS),
                Commands.either(
                    setSuperState(BACK_ALGAE_TOSS), 
                    setSuperState(FRONT_ALGAE_TOSS), 
                    () -> Robot.isRedAlliance() ^ (robotPoseSupplier.get().getRotation().getRadians() > Math.PI / 2d || robotPoseSupplier.get().getRotation().getRadians() < -Math.PI / 2d)
                ),
                Commands.waitUntil(() -> !algaeClaw.hasPiece()),
                setSuperState(READY_STOW)
            ),
            this::shouldEvadeReef
        );
    }

    public Command coralPlaceCommandAuto() {
        return Commands.sequence(
            outtakeCommand(),
            Commands.waitUntil(() -> !coralClaw.hasPiece())
        );
    }

    public Command netPlaceCommandAuto() {
        return Commands.sequence(
            outtakeCommand(),
            Commands.waitUntil(() -> !algaeClaw.hasPiece()),
            stopOuttakeCommand()
        );
    }

    public Command setSuperStateFromRemovalCommand(SuperState state) {
        return Commands.sequence(
            Commands.waitUntil(algaeClaw::hasPiece).onlyIf(() -> targetState.clawState.algaePercent > 0).withTimeout(2.0),
            new ScheduleCommand(setSuperState(state))
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

    @AutoLogOutput (key = "Subsystems/Superstructure/ShouldRunWristFast")
    public boolean shouldRunWristFast() {
        return (
            !algaeClaw.hasPiece() 
                || targetArmState == ArmState.NET 
                || targetArmState == ArmState.BACK_ALGAE_TOSS 
                || targetArmState == ArmState.FRONT_ALGAE_TOSS
                || targetArmState == ArmState.REEF_ALGAE_TOSS
        ) && !(
            targetArmState == ArmState.L2_ALGAE_EXIT 
                || targetArmState == ArmState.L3_ALGAE_EXIT
                || targetArmState == ArmState.ALGAE_CARRY
        );
    }

    @AutoLogOutput (key = "Subsystems/Superstructure/ShouldRunElevatorFast")
    public boolean shouldRunElevatorFast() {
        return (
            !algaeClaw.hasPiece() 
                || targetArmState == ArmState.NET
                || targetArmState == ArmState.REEF_ALGAE_TOSS
        ) && !(
            targetArmState == ArmState.L2_ALGAE_EXIT 
                || targetArmState == ArmState.L3_ALGAE_EXIT
                || targetArmState == ArmState.ALGAE_CARRY
        );
    }

    @AutoLogOutput (key = "Subsystems/Superstructure/ArmAtTargetPosition")
    public boolean armAtTargetPosition() {
        return elevator.atPosition(targetState.armState.elevatorPosition) && wrist.atPosition(targetState.armState.wristPosition);
    }

    @AutoLogOutput (key = "Subsystems/SuperStructure/StructureAtDesiredPosition")
    public boolean structureAtTargetPosition() {
        return armAtTargetPosition() && climb.atPosition(targetState.climbState.climbPosition);
    }

    @AutoLogOutput (key = "Subsystems/Superstructure/WristSafe")
    public boolean wristSafe() {
        Logger.recordOutput("Subsystems/Superstructure/WristVelocity", wrist.getVelocity());
        Logger.recordOutput("Subsystems/Superstructure/WristTarget", targetState.armState.wristPosition);
        Logger.recordOutput("Subsystems/Superstructure/WristPosition", wrist.getPosition());
        double velocitySignum = wristVelocitySignum();
        double positionSignum = wristPositionSignum();
        Logger.recordOutput("Subsystems/Superstructure/WristVelocitySignum", velocitySignum);
        Logger.recordOutput("Subsystems/Superstructure/WristPositionSignum", positionSignum);
        return ((wrist.atPosition(WristConstants.UNDER_THRESHOLD_RADIANS) || wrist.getPosition() > WristConstants.UNDER_THRESHOLD_RADIANS)) 
            // Wrist velocity direction equals desired direction of travel
            && velocitySignum == positionSignum;
    }

    public double wristVelocitySignum() {
        return Math.signum(MathUtil.applyDeadband(wrist.getVelocity(), WristConstants.VELOCITY_DEADBAND_RADIANS));
    }

    public double wristPositionSignum() {
        return Math.signum(MathUtil.applyDeadband(targetState.armState.wristPosition - wrist.getPosition(), WristConstants.POSITION_SIGNUM_DEADBAND_RADIANS));
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

    @AutoLogOutput (key = "Subsystems/Superstructure/TargetState/AlgaePercent")
    public double getAlgaePercent() {
        return targetState.clawState.algaePercent;
    }

}
