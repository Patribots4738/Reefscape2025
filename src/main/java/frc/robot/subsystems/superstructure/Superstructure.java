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

    @AutoLogOutput (key = "Subsystems/Superstructure/ArmPosition")
    private ArmPosition armPosition = ArmPosition.LOW_STOW;

    private final LoggedTunableNumber elevatorStow = new LoggedTunableNumber("Elevator/StowPostion", ElevatorConstants.STOW_POSITION_METERS);
    private final LoggedTunableNumber elevatorIntake = new LoggedTunableNumber("Elevator/IntakePosition", ElevatorConstants.INTAKE_POSITION_METERS);
    private final LoggedTunableNumber elevatorL1 = new LoggedTunableNumber("Elevator/L1Postition", ElevatorConstants.L1_POSITION_METERS);
    private final LoggedTunableNumber elevatorL2 = new LoggedTunableNumber("Elevator/L2Postition", ElevatorConstants.L2_POSITION_METERS);
    private final LoggedTunableNumber elevatorL3 = new LoggedTunableNumber("Elevator/L3Postition", ElevatorConstants.L3_POSITION_METERS);
    private final LoggedTunableNumber elevatorL4 = new LoggedTunableNumber("Elevator/L4Postition", ElevatorConstants.L4_POSITION_METERS);
    private final LoggedTunableNumber elevatorL2RemoveAlgae = new LoggedTunableNumber("Elevator/L4PostitionRemoveAlgae", ElevatorConstants.L2_POSITION_REMOVE_ALGAE);
    private final LoggedTunableNumber elevatorL3RemoveAlgae = new LoggedTunableNumber("Elevator/L4PostitionRemoveAlgae", ElevatorConstants.L3_POSITION_REMOVE_ALGAE);

    private final LoggedTunableNumber wristMinSafe = new LoggedTunableNumber("Wrist/MinSafePosition", WristConstants.MIN_SAFE_ANGLE_RADIANS);
    private final LoggedTunableNumber wristMaxSafe = new LoggedTunableNumber("Wrist/MaxSafePosition", WristConstants.MAX_SAFE_ANGLE_RADIANS);
    private final LoggedTunableNumber wristClimb = new LoggedTunableNumber("Wrist/ClimbPosition", WristConstants.MIN_SAFE_ANGLE_RADIANS);
    private final LoggedTunableNumber wristLowStow = new LoggedTunableNumber("Wrist/LowStowPostion", WristConstants.STOW_POSITION_RADIANS);
    private final LoggedTunableNumber wristUpStow = new LoggedTunableNumber("Wrist/UpStowPostion", WristConstants.MAX_ANGLE_RADIANS);
    private final LoggedTunableNumber wristIntake = new LoggedTunableNumber("Wrist/IntakePosition", WristConstants.INTAKE_POSITION_RADIANS);
    private final LoggedTunableNumber wristL1 = new LoggedTunableNumber("Wrist/L1Postition", WristConstants.L1_POSITION_RADIANS);
    private final LoggedTunableNumber wristL2 = new LoggedTunableNumber("Wrist/L2Postition", WristConstants.L2_POSITION_RADIANS);
    private final LoggedTunableNumber wristL3 = new LoggedTunableNumber("Wrist/L3Postition", WristConstants.L3_POSITION_RADIANS);
    private final LoggedTunableNumber wristL4 = new LoggedTunableNumber("Wrist/L4Postition", WristConstants.L4_POSITION_RADIANS);
    private final LoggedTunableNumber wristAlgaeGrab = new LoggedTunableNumber("Wrist/Algae", WristConstants.ALGAE_REMOVAL);
  
    private final LoggedTunableNumber coralClawPlaceTime = new LoggedTunableNumber("CoralClaw/PlaceTime", CoralClawConstants.PLACING_NAMED_COMMAND_TIME);
    private final LoggedTunableNumber algaeClawPlaceTime = new LoggedTunableNumber("AlgaeClaw/PlaceTime", AlgaeClawConstants.PLACING_NAMED_COMMAND_TIME);

    public Superstructure(AlgaeClaw algaeClaw, CoralClaw coralClaw, Elevator elevator, Wrist wrist, Climb climb, Supplier<Pose2d> robotPoseSupplier) {
        this.algaeClaw = algaeClaw;
        this.coralClaw = coralClaw;
        this.elevator = elevator;
        this.wrist = wrist;
        this.climb = climb;

        this.robotPoseSupplier = robotPoseSupplier;

        elevatorStow.onChanged(Commands.runOnce(() -> {
            ArmPosition.LOW_STOW.elevatorPose = elevatorStow.get();
            ArmPosition.UP_STOW.elevatorPose = elevatorStow.get();
        }).ignoringDisable(true));
        elevatorIntake.onChanged(Commands.runOnce(() -> ArmPosition.INTAKE.elevatorPose = elevatorIntake.get()).ignoringDisable(true));
        elevatorL1.onChanged(Commands.runOnce(() -> ArmPosition.L1.elevatorPose = elevatorL1.get()).ignoringDisable(true));
        elevatorL2.onChanged(Commands.runOnce(() -> ArmPosition.L2.elevatorPose = elevatorL2.get()).ignoringDisable(true));
        elevatorL3.onChanged(Commands.runOnce(() -> ArmPosition.L3.elevatorPose = elevatorL3.get()).ignoringDisable(true));
        elevatorL4.onChanged(Commands.runOnce(() -> ArmPosition.L4.elevatorPose = elevatorL4.get()).ignoringDisable(true));
        elevatorL2RemoveAlgae.onChanged(Commands.runOnce(() -> ArmPosition.L2_ALGAE.elevatorPose = elevatorL2RemoveAlgae.get()).ignoringDisable(true));
        elevatorL3RemoveAlgae.onChanged(Commands.runOnce(() -> ArmPosition.L3_ALGAE.elevatorPose = elevatorL3RemoveAlgae.get()).ignoringDisable(true));

        wristUpStow.onChanged(Commands.runOnce(() -> ArmPosition.UP_STOW.wristPose = wristUpStow.get()).ignoringDisable(true));
        wristLowStow.onChanged(Commands.runOnce(() -> ArmPosition.LOW_STOW.wristPose = wristLowStow.get()).ignoringDisable(true));
        wristClimb.onChanged(Commands.runOnce(() -> ArmPosition.CLIMB.wristPose = wristClimb.get()).ignoringDisable(true));
        wristIntake.onChanged(Commands.runOnce(() -> ArmPosition.INTAKE.wristPose = wristIntake.get()).ignoringDisable(true));
        wristL1.onChanged(Commands.runOnce(() -> ArmPosition.L1.wristPose = wristL1.get()).ignoringDisable(true));
        wristL2.onChanged(Commands.runOnce(() -> ArmPosition.L2.wristPose = wristL2.get()).ignoringDisable(true));
        wristL3.onChanged(Commands.runOnce(() -> ArmPosition.L3.wristPose = wristL3.get()).ignoringDisable(true));
        wristL4.onChanged(Commands.runOnce(() -> ArmPosition.L4.wristPose = wristL4.get()).ignoringDisable(true));
        wristAlgaeGrab.onChanged(Commands.runOnce(() -> {
            ArmPosition.L2_ALGAE.wristPose = wristAlgaeGrab.get();
            ArmPosition.L3_ALGAE.wristPose = wristAlgaeGrab.get();
        }).ignoringDisable(true));

    }

    public enum ArmPosition {
        UP_STOW  (ElevatorConstants.STOW_POSITION_METERS, WristConstants.MAX_ANGLE_RADIANS, false),
        LOW_STOW (ElevatorConstants.STOW_POSITION_METERS, WristConstants.STOW_POSITION_RADIANS, false),
        INTAKE   (ElevatorConstants.INTAKE_POSITION_METERS, WristConstants.INTAKE_POSITION_RADIANS, false),
        CLIMB    (ElevatorConstants.STOW_POSITION_METERS, WristConstants.MIN_SAFE_ANGLE_RADIANS, false),
        L2_ALGAE (ElevatorConstants.L2_POSITION_REMOVE_ALGAE, WristConstants.ALGAE_REMOVAL, false),
        L3_ALGAE (ElevatorConstants.L3_POSITION_REMOVE_ALGAE, WristConstants.ALGAE_REMOVAL, false),
        L1       (ElevatorConstants.L1_POSITION_METERS, WristConstants.L1_POSITION_RADIANS, true),
        L2       (ElevatorConstants.L2_POSITION_METERS, WristConstants.L2_POSITION_RADIANS, true),
        L3       (ElevatorConstants.L3_POSITION_METERS, WristConstants.L3_POSITION_RADIANS, true),
        L4       (ElevatorConstants.L4_POSITION_METERS, WristConstants.L4_POSITION_RADIANS, true);

        private double elevatorPose, wristPose;
        private boolean scoring;

        ArmPosition(double elevatorPose, double wristPose, boolean scoring) {
            this.elevatorPose = elevatorPose;
            this.wristPose = wristPose;
            this.scoring = scoring;
        }
    }

    public Command setArmPosition(ArmPosition position) {
        return 
            Commands.sequence(
                // Set arm state for logging
                Commands.runOnce(() -> {
                    this.armPosition = position;
                }),
                Commands.either(
                    // Move wrist to nearest transition pose, unless the arm was previously stowed up (which is a safe spot)
                    transitionWrist(() -> position.wristPose).onlyIf(() -> !wrist.atPosition(wristUpStow.get())), 
                    // Move wrist straight to target position
                    wrist.setPositionCommand(() -> position.wristPose), 
                    () -> 
                        // Only transition wrist if elevator needs to move in addition to other conditions
                        (shouldEvadeReef()
                            || position.wristPose < wristMinSafe.get()
                            || Robot.gameMode == GameMode.AUTONOMOUS) 
                        && !elevator.atPosition(position.elevatorPose)
                // Stop blocking sequence when wrist is in a safe position
                ).until(this::wristSafe),
                elevator.setPositionCommand(() -> position.elevatorPose),
                // Only effectual if wrist just transitioned
                wrist.setPositionCommand(() -> position.wristPose)
            );
    }

    public Command transitionWrist(DoubleSupplier targetWristPosition) {
        return wrist.setPositionCommand(() -> {
            double wristTransition;
            if (targetWristPosition.getAsDouble() >= wristMaxSafe.get()) {
                // Target pos is greater than safe range, transition in high end of safe range
                wristTransition = wristMaxSafe.get();
            } else if (targetWristPosition.getAsDouble() <= wristMinSafe.get()) {
                // Target pos is lower than safe range, transition in low end of safe range
                wristTransition = wristMinSafe.get();
            } else {
                // Target pos is in safe range, transition with target pos
                wristTransition = targetWristPosition.getAsDouble();
            }
            return wristTransition;
        });
    }

    public Command algaeIntakeCommand(BooleanSupplier continueIntakingSupplier)  {
        return Commands.sequence(
            setArmPosition(ArmPosition.L2_ALGAE),
            algaeClaw.intakeCommand(),
            Commands.waitUntil(() -> algaeClaw.hasPiece() || !continueIntakingSupplier.getAsBoolean()),
            algaeClaw.stopCommand(),
            setArmPosition(ArmPosition.L2_ALGAE)
        );
    }

    public Command coralIntakeCommand(BooleanSupplier continueIntakingSupplier) {
        return 
            Commands.sequence(
                setArmPosition(ArmPosition.INTAKE).until(elevator::atTargetPosition),
                coralClaw.intakeCommand(),
                Commands.waitUntil(() -> coralClaw.hasPiece() || !continueIntakingSupplier.getAsBoolean()),
                Commands.parallel(
                    coralClaw.stopCommand(),
                    setArmPosition(ArmPosition.LOW_STOW)
                )
            );
    }

    public Command coralAutoIntakeStartCommand(){
        return
            Commands.sequence(
                setArmPosition(ArmPosition.INTAKE),
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
            setArmPosition(ArmPosition.L2_ALGAE),
            algaeClaw.intakeCommand(),
            Commands.waitUntil(() -> !continueOuttakingSupplier.getAsBoolean()),
            algaeClaw.stopCommand(),
            setArmPosition(ArmPosition.L2_ALGAE)
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
                setArmPosition(ArmPosition.CLIMB),
                // Bring climb down to hard-stop
                climb.stowPositionCommand(),
                setArmPosition(ArmPosition.LOW_STOW)
            );
    }

    public Command climbReadyCommand() {
        return
            Commands.sequence(
                // Move arm down for low CG and out of way for clearance
                setArmPosition(ArmPosition.CLIMB),
                // Move climb to foot hard-stop
                climb.readyPositionCommand()
            );
    }

    public Command climbFinalCommand() {
        return
            Commands.sequence(
                // Move arm down for low CG and out of way for clearance
                setArmPosition(ArmPosition.CLIMB),
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
        return 
            // Wrist pos inside of safe range or at the edges within error bound
            wrist.getPosition() > wristMinSafe.get() && (wrist.getPosition() < wristMaxSafe.get() || !shouldEvadeReef()) 
            || wrist.atPosition(wristMinSafe.get())
            || wrist.atPosition(wristMaxSafe.get());
    }

    @AutoLogOutput (key = "Subsystems/Superstructure/ShouldEvadeReef")
    public boolean shouldEvadeReef() {
        return PoseCalculations.nearReef(robotPoseSupplier.get());
    }

}
