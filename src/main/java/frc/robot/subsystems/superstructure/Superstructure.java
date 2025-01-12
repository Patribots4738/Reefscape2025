package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.Constants.WristConstants;
import frc.robot.util.custom.LoggedTunableNumber;
import frc.robot.subsystems.superstructure.climb.Climb;

public class Superstructure extends SubsystemBase {
    
    private Claw claw;
    private Elevator elevator;
    private Wrist wrist;
    private Climb climb;

    private PlacePosition currentPosition = PlacePosition.STOW;

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

        elevatorStow.onChanged(runOnce(() -> PlacePosition.STOW.elevatorPose = elevatorStow.get()));
        elevatorIntake.onChanged(runOnce(() -> PlacePosition.INTAKE.elevatorPose = elevatorIntake.get()));
        elevatorL1.onChanged(runOnce(() -> PlacePosition.L1.elevatorPose = elevatorL1.get()));
        elevatorL2.onChanged(runOnce(() -> PlacePosition.L2.elevatorPose = elevatorL2.get()));
        elevatorL3.onChanged(runOnce(() -> PlacePosition.L3.elevatorPose = elevatorL3.get()));
        elevatorL4.onChanged(runOnce(() -> PlacePosition.L4.elevatorPose = elevatorL4.get()));

        wristStow.onChanged(runOnce(() -> PlacePosition.STOW.wristPose = wristStow.get()));
        wristIntake.onChanged(runOnce(() -> PlacePosition.INTAKE.wristPose = wristIntake.get()));
        wristL1.onChanged(runOnce(() -> PlacePosition.L1.wristPose = wristL1.get()));
        wristL2.onChanged(runOnce(() -> PlacePosition.L2.wristPose = wristL2.get()));
        wristL3.onChanged(runOnce(() -> PlacePosition.L3.wristPose = wristL3.get()));
        wristL4.onChanged(runOnce(() -> PlacePosition.L4.wristPose = wristL4.get()));
    }

    public enum PlacePosition {
        INTAKE (ElevatorConstants.INTAKE_POSITION_METERS, WristConstants.INTAKE_POSITION_RADIANS),
        STOW   (ElevatorConstants.STOW_POSITION_METERS, WristConstants.STOW_POSITION_RADIANS),
        L1     (ElevatorConstants.L1_POSITION_METERS, WristConstants.L1_POSITION_RADIANS),
        L2     (ElevatorConstants.L2_POSITION_METERS, WristConstants.L2_POSITION_RADIANS),
        L3     (ElevatorConstants.L3_POSITION_METERS, WristConstants.L3_POSITION_RADIANS),
        L4     (ElevatorConstants.L4_POSITION_METERS, WristConstants.L4_POSITION_RADIANS);

        private double elevatorPose, wristPose;

        PlacePosition(double elevatorPose, double wristPose) {
            this.elevatorPose = elevatorPose;
            this.wristPose = wristPose;
        }

    }

    public Command goToPlacement() {
        return 
            run(() -> 
                elevator.setPositionCommand(() -> currentPosition.elevatorPose)
                    .alongWith(wrist.setPositionCommand(() -> currentPosition.wristPose)).schedule());
    }

    public Command changePlacement(PlacePosition newPosition) {
        return runOnce(() -> this.currentPosition = newPosition);
    }

    @Override
    public void periodic() {
        System.out.println(currentPosition + ": " + currentPosition.elevatorPose + " " + currentPosition.wristPose);
    }
}
