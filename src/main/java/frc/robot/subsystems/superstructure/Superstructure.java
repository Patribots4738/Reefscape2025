package frc.robot.subsystems.superstructure;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;
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

    private final LoggedTunableNumber wristTransition = new LoggedTunableNumber("Wrist/TransitionPosition", WristConstants.TRANSITION_RADIANS);
    private final LoggedTunableNumber coralClawPlaceTime = new LoggedTunableNumber("CoralClaw/PlaceTime", CoralClawConstants.PLACING_NAMED_COMMAND_TIME);

    public final LoggedSuperState STOW;
    public final LoggedSuperState INTAKE;
    public final LoggedSuperState L1;
    public final LoggedSuperState L2;
    public final LoggedSuperState L3;
    public final LoggedSuperState L4;
    public final LoggedSuperState L1_PLACE;
    public final LoggedSuperState L2_PLACE;
    public final LoggedSuperState L3_PLACE;
    public final LoggedSuperState L4_PLACE;
    public final LoggedSuperState L2_ALGAE;
    public final LoggedSuperState L3_ALGAE;
    public final LoggedSuperState L2_ALGAE_IN;
    public final LoggedSuperState L3_ALGAE_IN;
    public final LoggedSuperState CLIMB_READY;
    public final LoggedSuperState CLIMB_FINAL;

    private SuperState targetState;

    public Superstructure(AlgaeClaw algaeClaw, CoralClaw coralClaw, Elevator elevator, Wrist wrist, Climb climb, Supplier<Pose2d> robotPoseSupplier) {
        this.algaeClaw = algaeClaw;
        this.coralClaw = coralClaw;
        this.elevator = elevator;
        this.wrist = wrist;
        this.climb = climb;

        this.robotPoseSupplier = robotPoseSupplier;

        STOW = new LoggedSuperState("STOW", ArmState.STOW, ClimbState.STOW, ClawState.STOP);
        INTAKE = new LoggedSuperState("INTAKE", ArmState.INTAKE, ClimbState.STOW, ClawState.CORAL_IN);
        L1 = new LoggedSuperState("L1", ArmState.L1, ClimbState.STOW, ClawState.STOP);
        L2 = new LoggedSuperState("L2", ArmState.L2, ClimbState.STOW, ClawState.STOP);
        L3 = new LoggedSuperState("L3", ArmState.L3, ClimbState.STOW, ClawState.STOP);
        L4 = new LoggedSuperState("L4", ArmState.L4, ClimbState.STOW, ClawState.STOP);
        L1_PLACE = new LoggedSuperState("L1", ArmState.L1, ClimbState.STOW, ClawState.CORAL_OUT);
        L2_PLACE = new LoggedSuperState("L2", ArmState.L2, ClimbState.STOW, ClawState.CORAL_OUT);
        L3_PLACE = new LoggedSuperState("L3", ArmState.L3, ClimbState.STOW, ClawState.CORAL_OUT);
        L4_PLACE = new LoggedSuperState("L4", ArmState.L4, ClimbState.STOW, ClawState.CORAL_OUT);
        L2_ALGAE = new LoggedSuperState("L2_ALGAE", ArmState.L2_ALGAE, ClimbState.STOW, ClawState.STOP);
        L3_ALGAE = new LoggedSuperState("L3_ALGAE", ArmState.L3_ALGAE, ClimbState.STOW, ClawState.STOP);
        L2_ALGAE_IN = new LoggedSuperState("L2_ALGAE_IN", ArmState.L2_ALGAE, ClimbState.STOW, ClawState.ALGAE_IN);
        L3_ALGAE_IN = new LoggedSuperState("L2_ALGAE_IN", ArmState.L2_ALGAE, ClimbState.STOW, ClawState.ALGAE_IN);
        CLIMB_READY = new LoggedSuperState("CLIMB_READY", ArmState.CLIMB, ClimbState.READY, ClawState.STOP);
        CLIMB_FINAL = new LoggedSuperState("CLIMB_FINAL", ArmState.CLIMB, ClimbState.FINAL, ClawState.STOP);

        targetState = STOW;
    }

    public enum ArmState {

        STOW (ElevatorConstants.STOW_POSITION_METERS, WristConstants.STOW_POSITION_RADIANS),
        INTAKE (ElevatorConstants.INTAKE_POSITION_METERS, WristConstants.INTAKE_POSITION_RADIANS),
        L1 (ElevatorConstants.L1_POSITION_METERS, WristConstants.L1_POSITION_RADIANS),
        L2 (ElevatorConstants.L3_POSITION_METERS, WristConstants.L3_POSITION_RADIANS),
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

    };

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

    public Command setSuperState(SuperState nextState) {
        return Commands.sequence(
            Commands.runOnce(() -> this.targetState = nextState),
            stopClaw(),
            Commands.either(
                Commands.either(
                    Commands.sequence(
                        setArmState(ArmState.CLIMB),
                        setClimbState(nextState.climbState),
                        moveToFinalArm(nextState)
                    ), 
                    Commands.sequence(
                        moveToFinalArm(nextState),
                        setClimbState(nextState.climbState)
                    ), 
                    () -> nextState.armState.wristPosition < WristConstants.CLIMB_RADIANS
                ), 
                moveToFinalArm(nextState), 
                () -> !climb.atPosition(nextState.climbState.climbPosition)
            )
        );
    }

    public Command moveToFinalArm(SuperState nextState) {
        return Commands.sequence(
            Commands.deadline(
                setArmState(nextState.armState),
                Commands.sequence(
                    Commands.waitUntil(nextState.coralInterruptSupplier),
                    coralClaw.setPercentCommand(() -> nextState.clawState.coralPercent)
                ),
                Commands.sequence(
                    Commands.waitUntil(nextState.algaeInterruptSupplier),
                    coralClaw.setPercentCommand(() -> nextState.clawState.algaePercent)
                )
            ),
            Commands.parallel(
                coralClaw.setPercentCommand(() -> nextState.clawState.coralPercent),
                algaeClaw.setPercentCommand(() -> nextState.clawState.algaePercent)
            )
        );
    }

    public Command stopClaw() {
        return Commands.parallel(
            coralClaw.stopCommand(),
            algaeClaw.stopCommand()
        );
    }

    public Command setArmState(ArmState state) {
        return 
            Commands.sequence(
                Commands.either(
                    // Move wrist to nearest transition pose, unless the arm was previously stowed up (which is a safe spot)
                    transitionWrist(() -> state.wristPosition), 
                    // Move wrist straight to target position
                    wrist.setPositionCommand(() -> state.wristPosition), 
                    () -> 
                        // Only transition wrist if elevator needs to move in addition to other conditions
                        (shouldEvadeReef()
                            || state.wristPosition < wristTransition.get()
                            || Robot.gameMode == GameMode.AUTONOMOUS) 
                        && !elevator.atPosition(state.elevatorPosition)
                // Stop blocking sequence when wrist is in a safe position
                ).until(this::wristSafe),
                elevator.setPositionCommand(() -> state.elevatorPosition),
                // Only effectual if wrist just transitioned
                wrist.setPositionCommand(() -> state.wristPosition)
            );
    }

    public Command setClimbState(ClimbState state) {
        return climb.setPositionCommand(() -> state.climbPosition);
    }

    public Command transitionWrist(DoubleSupplier targetWristPosition) {
        return wrist.setPositionCommand(wristTransition::get);
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

    public Command coralIntakeCommand(BooleanSupplier continueIntakingSupplier) {
        return 
            Commands.sequence(
                setSuperState(INTAKE),
                Commands.waitUntil(() -> !continueIntakingSupplier.getAsBoolean()),
                setSuperState(STOW)
            );
    }

    public Command coralAutoIntakeStartCommand(){
        return setSuperState(INTAKE);
    }

    public Command coralAutoIntakeStopCommand(){
        return setSuperState(STOW);
    }

    public SuperState getPlacementState() {
        // Derive next state based on current arm target
        return switch (targetState.armState) {
            case L2 -> L2_PLACE;
            case L3 -> L3_PLACE;
            case L4 -> L4_PLACE;
            default -> L1_PLACE;
        };
    }

    public SuperState getStopState() {
        // Derive next state based on current arm target
        // I know the L2 -> L2 thing looks weird, its basically converting the current target ArmState to the next overall SuperState
        return switch (targetState.armState) {
            case L2 -> L2;
            case L3 -> L3;
            case L4 -> L4;
            default -> L1;
        };
    }

    public Command coralPlaceCommand(BooleanSupplier continueOuttakingSupplier) {
        return
            Commands.sequence(
                setSuperState(getPlacementState()),
                Commands.waitUntil(() -> !continueOuttakingSupplier.getAsBoolean()),
                setSuperState(getStopState())
            );
    }

    public Command coralAutoPlaceCommand() {
        return
            Commands.sequence(
                setSuperState(getPlacementState()),
                Commands.waitSeconds(coralClawPlaceTime.get()),
                setSuperState(getStopState())
            );
    }

    public Command climbReadyCommand() {
        return setSuperState(CLIMB_READY);
    }

    public Command climbFinalCommand() {
        return setSuperState(CLIMB_FINAL);
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
        return wrist.atPosition(wristTransition.get());
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
        return targetState.armState;
    }

    @AutoLogOutput (key = "Subsystems/Superstructure/TargetState/ClimbState")
    public ClimbState getTargetClimbState() {
        return targetState.climbState;
    }

    @AutoLogOutput (key = "Subsystems/Superstructure/TargetState/ClawState")
    public ClawState getTargetClawState() {
        return targetState.clawState;
    }

}
