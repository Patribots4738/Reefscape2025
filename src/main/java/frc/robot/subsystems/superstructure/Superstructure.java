package frc.robot.subsystems.superstructure;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.Constants.WristConstants;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.custom.LoggedTunableNumber;
import frc.robot.subsystems.superstructure.climb.Climb;

public class Superstructure {
    
    private final Claw claw;
    private final Elevator elevator;
    private final Wrist wrist;
    private final Climb climb;

    private final LoggedTunableNumber elevatorStow = new LoggedTunableNumber("Elevator/StowPostion", ElevatorConstants.STOW_POSITION_METERS);
    private final LoggedTunableNumber elevatorIntake = new LoggedTunableNumber("Elevator/IntakePosition", ElevatorConstants.INTAKE_POSITION_METERS);
    private final LoggedTunableNumber elevatorL1 = new LoggedTunableNumber("Elevator/L1Postition", ElevatorConstants.L1_POSITION_METERS);
    private final LoggedTunableNumber elevatorL2 = new LoggedTunableNumber("Elevator/L2Postition", ElevatorConstants.L2_POSITION_METERS);
    private final LoggedTunableNumber elevatorL3 = new LoggedTunableNumber("Elevator/L3Postition", ElevatorConstants.L3_POSITION_METERS);
    private final LoggedTunableNumber elevatorL4 = new LoggedTunableNumber("Elevator/L4Postition", ElevatorConstants.L4_POSITION_METERS);

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
        elevatorL1.onChanged(Commands.runOnce(() -> ArmPosition.L1.elevatorPose = elevatorL1.get()).ignoringDisable(true));
        elevatorL2.onChanged(Commands.runOnce(() -> ArmPosition.L2.elevatorPose = elevatorL2.get()).ignoringDisable(true));
        elevatorL3.onChanged(Commands.runOnce(() -> ArmPosition.L3.elevatorPose = elevatorL3.get()).ignoringDisable(true));
        elevatorL4.onChanged(Commands.runOnce(() -> ArmPosition.L4.elevatorPose = elevatorL4.get()).ignoringDisable(true));

        wristStow.onChanged(Commands.runOnce(() -> ArmPosition.STOW.wristPose = wristStow.get()).ignoringDisable(true));
        wristIntake.onChanged(Commands.runOnce(() -> ArmPosition.INTAKE.wristPose = wristIntake.get()).ignoringDisable(true));
        wristL1.onChanged(Commands.runOnce(() -> ArmPosition.L1.wristPose = wristL1.get()).ignoringDisable(true));
        wristL2.onChanged(Commands.runOnce(() -> ArmPosition.L2.wristPose = wristL2.get()).ignoringDisable(true));
        wristL3.onChanged(Commands.runOnce(() -> ArmPosition.L3.wristPose = wristL3.get()).ignoringDisable(true));
        wristL4.onChanged(Commands.runOnce(() -> ArmPosition.L4.wristPose = wristL4.get()).ignoringDisable(true));
    }

    public enum MovementOrder {
        WRIST_FIRST,
        ELEVATOR_FIRST,
        SIMULTANEOUS
    }

    public enum ArmPosition {
        
        INTAKE (ElevatorConstants.INTAKE_POSITION_METERS, WristConstants.INTAKE_POSITION_RADIANS),
        STOW   (ElevatorConstants.STOW_POSITION_METERS, WristConstants.STOW_POSITION_RADIANS),
        L1     (ElevatorConstants.L1_POSITION_METERS, WristConstants.L1_POSITION_RADIANS),
        L2     (ElevatorConstants.L2_POSITION_METERS, WristConstants.L2_POSITION_RADIANS),
        L3     (ElevatorConstants.L3_POSITION_METERS, WristConstants.L3_POSITION_RADIANS),
        L4     (ElevatorConstants.L4_POSITION_METERS, WristConstants.L4_POSITION_RADIANS);

        private double elevatorPose, wristPose;

        ArmPosition(double elevatorPose, double wristPose) {
            this.elevatorPose = elevatorPose;
            this.wristPose = wristPose;
        }
    }

    public Command setArmPosition(ArmPosition position, MovementOrder order) {
        return 
            Commands.either(
                elevator.setPositionCommand(() -> position.elevatorPose)
                    .alongWith(wrist.setPositionCommand(() -> position.wristPose)),
                Commands.either(
                    elevator.setPositionCommand(() -> position.elevatorPose)
                        .andThen(wrist.setPositionCommand(() -> position.wristPose)),
                    wrist.setPositionCommand(() -> position.wristPose)
                        .andThen(elevator.setPositionCommand(() -> position.elevatorPose)),
                    () -> order == MovementOrder.ELEVATOR_FIRST
                ), 
                () -> order == MovementOrder.SIMULTANEOUS
            );
    }

    public Command setArmPosition(ArmPosition position) {
        return setArmPosition(position, MovementOrder.SIMULTANEOUS);
    }

    public boolean armAtTargetPosition() {
        return elevator.atTargetPosition() && wrist.atTargetPosition();
    }

    public Command stowCommand() {
        return
            Commands.sequence(
                setArmPosition(ArmPosition.STOW)
            );
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

    public Command autoIntakeStartCommand(){
        return
            Commands.sequence(
                setArmPosition(ArmPosition.INTAKE),
                claw.intakeCommand()
            );
    }

    public Command autoIntakeStopCommand(){
        return
            Commands.sequence(
                claw.stopCommand(),
                setArmPosition(ArmPosition.STOW)
            );
    }

    public Command placeCommand(BooleanSupplier continueOuttakingSupplier) {
        return
            Commands.sequence(
                Commands.waitUntil(() -> elevator.atTargetPosition() && wrist.atTargetPosition()),
                claw.outtakeCommand(),
                Commands.waitUntil(() -> continueOuttakingSupplier.getAsBoolean()),
                claw.stopCommand(),
                stowCommand()
            );
    }

    public Command autoPlaceCommand(double time) {
        return
            Commands.sequence(
                Commands.waitUntil(() -> elevator.atTargetPosition() && wrist.atTargetPosition()),
                claw.outtakeCommand(),
                claw.stopCommand(),
                stowCommand()
            );
    }

    public Command outtakeCommand(BooleanSupplier continueOuttakingSupplier) {
        return
            Commands.sequence(
                setArmPosition(ArmPosition.L1),
                placeCommand(continueOuttakingSupplier)
            );
    }




}
