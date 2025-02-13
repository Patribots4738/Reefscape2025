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
    private final LoggedTunableNumber algaeClawPlaceTime = new LoggedTunableNumber("AlgaeClaw/PlaceTime", AlgaeClawConstants.PLACING_NAMED_COMMAND_TIME);

    private final LoggedSuperState STOW;
    private final LoggedSuperState INTAKE;
    private final LoggedSuperState L1;
    private final LoggedSuperState L2;
    private final LoggedSuperState L3;
    private final LoggedSuperState L4;
    private final LoggedSuperState L1_PLACE;
    private final LoggedSuperState L2_PLACE;
    private final LoggedSuperState L3_PLACE;
    private final LoggedSuperState L4_PLACE;
    private final LoggedSuperState L2_ALGAE;
    private final LoggedSuperState L3_ALGAE;
    private final LoggedSuperState CLIMB_READY;
    private final LoggedSuperState CLIMB_FINAL;

    private SuperState currentState;
    private SuperState nextState;

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
        L2_ALGAE = new LoggedSuperState("L2_ALGAE", ArmState.L2_ALGAE, ClimbState.STOW, ClawState.ALGAE_IN);
        L3_ALGAE = new LoggedSuperState("L3_ALGAE", ArmState.L3_ALGAE, ClimbState.STOW, ClawState.ALGAE_IN);
        CLIMB_READY = new LoggedSuperState("CLIMB_READY", ArmState.CLIMB, ClimbState.READY, ClawState.STOP);
        CLIMB_FINAL = new LoggedSuperState("CLIMB_FINAL", ArmState.CLIMB, ClimbState.FINAL, ClawState.STOP);

        currentState = STOW;
        nextState = STOW;
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
                () -> climb.getPosition() != nextState.climbState.climbPosition
            )
        );
    }

    public Command moveToFinalArm(SuperState nextState) {
        return Commands.sequence(
            Commands.parallel(
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
            setArmState(ArmState.L2_ALGAE),
            algaeClaw.intakeCommand(),
            Commands.waitUntil(() -> algaeClaw.hasPiece() || !continueIntakingSupplier.getAsBoolean()),
            algaeClaw.stopCommand()
        );
    }

    public Command algaeL3Command(BooleanSupplier continueIntakingSupplier)  {
        return Commands.sequence(
            setArmState(ArmState.L3_ALGAE),
            algaeClaw.intakeCommand(),
            Commands.waitUntil(() -> algaeClaw.hasPiece() || !continueIntakingSupplier.getAsBoolean()),
            algaeClaw.stopCommand()
        );
    }

    public Command coralIntakeCommand(BooleanSupplier continueIntakingSupplier) {
        return 
            Commands.sequence(
                Commands.parallel(
                    setArmState(ArmState.INTAKE),
                    coralClaw.intakeCommand()
                ),
                Commands.waitUntil(() -> coralClaw.hasPiece() || !continueIntakingSupplier.getAsBoolean()),
                Commands.parallel(
                    coralClaw.stopCommand(),
                    setArmState(ArmState.STOW)
                )
            );
    }

    public Command coralAutoIntakeStartCommand(){
        return
            Commands.sequence(
                setArmState(ArmState.INTAKE),
                coralClaw.intakeCommand()
            );
    }

    public Command coralAutoIntakeStopCommand(){
        return coralClaw.stopCommand();
    }

    public Command coralPlaceCommand(BooleanSupplier continueOuttakingSupplier) {
        return
            Commands.sequence(
                // Commands.either(
                //     // Wait until arm stopped
                //     Commands.waitUntil(this::armAtTargetPosition), 
                //     // If arm not currently at scoring pos, go to L1
                //     setArmPosition(ArmPosition.L1), 
                //     () -> this.armPosition.scoring
                // ),
                coralClaw.outtakeCommand(),
                Commands.waitUntil(() -> !continueOuttakingSupplier.getAsBoolean()),
                coralClaw.stopCommand()
                // setArmPosition(ArmPosition.LOW_STOW)
            );
    }

    public Command algaePlaceCommand(BooleanSupplier continueOuttakingSupplier) {
        return Commands.sequence(
            setArmState(ArmState.L2_ALGAE),
            algaeClaw.intakeCommand(),
            Commands.waitUntil(() -> !continueOuttakingSupplier.getAsBoolean()),
            algaeClaw.stopCommand(),
            setArmState(ArmState.L2_ALGAE)
        );
    }

    public Command coralAutoPlaceCommand() {
        return
            Commands.sequence(
                Commands.waitUntil(this::armAtTargetPosition),
                coralClaw.outtakeTimeCommand(coralClawPlaceTime.get())
            );
    }

    public Command algaeAutoPlaceCommand() {
        return
            Commands.sequence(
                Commands.waitUntil(this::armAtTargetPosition),
                algaeClaw.outtakeTimeCommand(algaeClawPlaceTime.get())
            );
    }

    public Command climbStowCommand() {
        return
            Commands.sequence(
                // Move arm out of way for clearance
                setArmState(ArmState.CLIMB),
                // Bring climb down to hard-stop
                climb.stowPositionCommand(),
                setArmState(ArmState.STOW)
            );
    }

    public Command climbReadyCommand() {
        return
            Commands.sequence(
                // Move arm down for low CG and out of way for clearance
                setArmState(ArmState.CLIMB),
                // Move climb to foot hard-stop
                climb.readyPositionCommand()
            );
    }

    public Command climbFinalCommand() {
        return
            Commands.sequence(
                // Move arm down for low CG and out of way for clearance
                setArmState(ArmState.CLIMB),
                // Move climb & cage to flat position
                climb.finalPositionCommand()
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
        return elevator.atTargetPosition() && wrist.atTargetPosition();
    }

    @AutoLogOutput (key = "Subsystems/Superstructure/WristSafe")
    public boolean wristSafe() {
        return wrist.atPosition(wristTransition.get());
    }

    @AutoLogOutput (key = "Subsystems/Superstructure/ShouldEvadeReef")
    public boolean shouldEvadeReef() {
        return PoseCalculations.nearReef(robotPoseSupplier.get());
    }

}
