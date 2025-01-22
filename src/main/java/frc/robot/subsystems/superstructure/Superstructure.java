package frc.robot.subsystems.superstructure;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.Constants.WristConstants;
import frc.robot.util.custom.LoggedTunableNumber;
import frc.robot.subsystems.superstructure.climb.Climb;

public class Superstructure {
    
    private final Claw claw;
    private final Elevator elevator;
    private final Wrist wrist;
    private final Climb climb;

    private ArmPosition armPosition = ArmPosition.STOW;

    private final LoggedTunableNumber elevatorStow = new LoggedTunableNumber("Elevator/StowPostion", ElevatorConstants.STOW_POSITION_METERS);
    private final LoggedTunableNumber elevatorIntake = new LoggedTunableNumber("Elevator/IntakePosition", ElevatorConstants.INTAKE_POSITION_METERS);
    private final LoggedTunableNumber elevatorPrepL1 = new LoggedTunableNumber("Elevator/L1Postition", ElevatorConstants.L1_PREP_POSITION_METERS);
    private final LoggedTunableNumber elevatorPrepL2 = new LoggedTunableNumber("Elevator/L2Postition", ElevatorConstants.L2_PREP_POSITION_METERS);
    private final LoggedTunableNumber elevatorPrepL3 = new LoggedTunableNumber("Elevator/L3Postition", ElevatorConstants.L3_PREP_POSITION_METERS);
    private final LoggedTunableNumber elevatorPrepL4 = new LoggedTunableNumber("Elevator/L4Postition", ElevatorConstants.L4_PREP_POSITION_METERS);
    private final LoggedTunableNumber elevatorPlaceL1 = new LoggedTunableNumber("Elevator/L1Postition", ElevatorConstants.L1_PREP_POSITION_METERS);
    private final LoggedTunableNumber elevatorPlaceL2 = new LoggedTunableNumber("Elevator/L2Postition", ElevatorConstants.L2_PREP_POSITION_METERS);
    private final LoggedTunableNumber elevatorPlaceL3 = new LoggedTunableNumber("Elevator/L3Postition", ElevatorConstants.L3_PREP_POSITION_METERS);
    private final LoggedTunableNumber elevatorPlaceL4 = new LoggedTunableNumber("Elevator/L4Postition", ElevatorConstants.L4_PREP_POSITION_METERS);

    private final LoggedTunableNumber wristTransition = new LoggedTunableNumber("Wrist/TransitionPosition", WristConstants.TRANSITION_POSITION_RADIANS);
    private final LoggedTunableNumber wristStow = new LoggedTunableNumber("Wrist/StowPostion", WristConstants.STOW_POSITION_RADIANS);
    private final LoggedTunableNumber wristIntake = new LoggedTunableNumber("Wrist/IntakePosition", WristConstants.INTAKE_POSITION_RADIANS);
    private final LoggedTunableNumber wristL1 = new LoggedTunableNumber("Wrist/L1Postition", WristConstants.L1_POSITION_RADIANS);
    private final LoggedTunableNumber wristL2 = new LoggedTunableNumber("Wrist/L2Postition", WristConstants.L2_POSITION_RADIANS);
    private final LoggedTunableNumber wristL3 = new LoggedTunableNumber("Wrist/L3Postition", WristConstants.L3_POSITION_RADIANS);
    private final LoggedTunableNumber wristL4 = new LoggedTunableNumber("Wrist/L4Postition", WristConstants.L4_POSITION_RADIANS);

    public Superstructure(Claw claw, Elevator elevator, Wrist wrist, Climb climb) {
        this.claw = claw;
        this.elevator = elevator;
        this.wrist = wrist;
        this.climb = climb;

        elevatorStow.onChanged(Commands.runOnce(() -> ArmPosition.STOW.elevatorPose = elevatorStow.get()).ignoringDisable(true));
        elevatorIntake.onChanged(Commands.runOnce(() -> ArmPosition.INTAKE.elevatorPose = elevatorIntake.get()).ignoringDisable(true));
        elevatorPrepL1.onChanged(Commands.runOnce(() -> ArmPosition.L1_PREP.elevatorPose = elevatorPrepL1.get()).ignoringDisable(true));
        elevatorPrepL2.onChanged(Commands.runOnce(() -> ArmPosition.L2_PREP.elevatorPose = elevatorPrepL2.get()).ignoringDisable(true));
        elevatorPrepL3.onChanged(Commands.runOnce(() -> ArmPosition.L3_PREP.elevatorPose = elevatorPrepL3.get()).ignoringDisable(true));
        elevatorPrepL4.onChanged(Commands.runOnce(() -> ArmPosition.L4_PREP.elevatorPose = elevatorPrepL4.get()).ignoringDisable(true));

        wristStow.onChanged(Commands.runOnce(() -> ArmPosition.STOW.wristPose = wristStow.get()).ignoringDisable(true));
        wristIntake.onChanged(Commands.runOnce(() -> ArmPosition.INTAKE.wristPose = wristIntake.get()).ignoringDisable(true));
        wristL1.onChanged(Commands.runOnce(() -> ArmPosition.L1_PREP.wristPose = wristL1.get()).ignoringDisable(true));
        wristL2.onChanged(Commands.runOnce(() -> ArmPosition.L2_PREP.wristPose = wristL2.get()).ignoringDisable(true));
        wristL3.onChanged(Commands.runOnce(() -> ArmPosition.L3_PREP.wristPose = wristL3.get()).ignoringDisable(true));
        wristL4.onChanged(Commands.runOnce(() -> ArmPosition.L4_PREP.wristPose = wristL4.get()).ignoringDisable(true));
    }

    public enum ArmPosition {
        
        INTAKE (ElevatorConstants.INTAKE_POSITION_METERS, WristConstants.INTAKE_POSITION_RADIANS),
        STOW   (ElevatorConstants.STOW_POSITION_METERS, WristConstants.STOW_POSITION_RADIANS),
        L1_PREP     (ElevatorConstants.L1_PREP_POSITION_METERS, WristConstants.L1_POSITION_RADIANS),
        L1_PLACE     (ElevatorConstants.L1_PREP_POSITION_METERS, WristConstants.L1_POSITION_RADIANS),
        L2_PREP     (ElevatorConstants.L2_PREP_POSITION_METERS, WristConstants.L2_POSITION_RADIANS),
        L2_PLACE     (ElevatorConstants.L2_PREP_POSITION_METERS, WristConstants.L2_POSITION_RADIANS),
        L3_PREP     (ElevatorConstants.L3_PREP_POSITION_METERS, WristConstants.L3_POSITION_RADIANS),
        L3_PLACE     (ElevatorConstants.L3_PREP_POSITION_METERS, WristConstants.L3_POSITION_RADIANS),
        L4_PREP     (ElevatorConstants.L4_PREP_POSITION_METERS, WristConstants.L4_POSITION_RADIANS),
        L4_PLACE     (ElevatorConstants.L4_PREP_POSITION_METERS, WristConstants.L4_POSITION_RADIANS);

        private double elevatorPose, wristPose;

        ArmPosition(double elevatorPose, double wristPose) {
            this.elevatorPose = elevatorPose;
            this.wristPose = wristPose;
        }
    }

    public Command setArmPosition(ArmPosition position) {
        return 
            Commands.sequence(
                Commands.runOnce(() -> this.armPosition = position),
                Commands.race(
                    wrist.setPositionCommand(wristTransition::get),
                    Commands.waitUntil(this::wristSafe)
                ).onlyIf(() -> !elevator.atPosition(position.elevatorPose)),
                elevator.setPositionCommand(() -> position.elevatorPose),
                wrist.setPositionCommand(() -> position.wristPose)
            );
    }

    public boolean armAtTargetPosition() {
        return elevator.atTargetPosition() && wrist.atTargetPosition();
    }

    public boolean wristSafe() {
        return wrist.getPosition() < 2 * Math.PI / 3.0 && wrist.getPosition() > Math.PI / 3.0;
    }

    public ArmPosition getPlacement() {
        return switch (armPosition) {
            case L1_PREP -> ArmPosition.L1_PLACE;
            case L2_PREP -> ArmPosition.L2_PLACE;
            case L3_PREP -> ArmPosition.L3_PLACE;
            case L4_PREP -> ArmPosition.L4_PLACE;
            default -> ArmPosition.STOW;
        };
    }

    public ArmPosition getReset() {
        return switch (armPosition) {
            case L1_PLACE -> ArmPosition.L1_PREP;
            case L2_PLACE -> ArmPosition.L2_PREP;
            case L3_PLACE -> ArmPosition.L3_PREP;
            case L4_PLACE -> ArmPosition.L4_PREP;
            default -> ArmPosition.STOW;
        };
    }

    public Command stowCommand() {
        return
            setArmPosition(ArmPosition.STOW);
    }

    public Command intakeCommand(BooleanSupplier continueIntakingSupplier) {
        return 
            Commands.sequence(
                setArmPosition(ArmPosition.INTAKE),
                claw.intakeCommand(),
                Commands.waitUntil(() -> claw.hasPiece() || !continueIntakingSupplier.getAsBoolean()),
                claw.holdCommand(),
                stowCommand()
            );
    }

    public Command placeCommand(BooleanSupplier continueOuttakingSupplier) {
        return
            Commands.sequence(
                Commands.waitUntil(() -> elevator.atTargetPosition() && wrist.atTargetPosition()),
                setArmPosition(getPlacement()),
                claw.outtakeCommand(),
                Commands.waitUntil(() -> continueOuttakingSupplier.getAsBoolean()),
                claw.stopCommand(),
                setArmPosition(getReset()),
                stowCommand()
            );
    }

    public Command outtakeCommand(BooleanSupplier continueOuttakingSupplier) {
        return
            Commands.sequence(
                setArmPosition(ArmPosition.L1_PREP),
                placeCommand(continueOuttakingSupplier)
            );
    }

}
