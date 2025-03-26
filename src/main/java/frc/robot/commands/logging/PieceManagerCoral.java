package frc.robot.commands.logging;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.calc.PoseCalculations;

/**
 * Implementation of PieceManagerIO for coral pieces
 */
public class PieceManagerCoral implements PieceManagerIO {
    private final BooleanSupplier hasPieceSupplier;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final DoubleSupplier elevatorHeightSupplier;
    
    // Constants
    private static final double HIDDEN_Z_OFFSET = -0.05;
    private static final double PIECE_ANIMATION_DURATION = 0.1;
    
    /**
     * Create a new coral piece manager
     */
    public PieceManagerCoral(BooleanSupplier hasPieceSupplier, 
                           Supplier<Pose2d> robotPoseSupplier,
                           DoubleSupplier elevatorHeightSupplier) {
        this.hasPieceSupplier = hasPieceSupplier;
        this.robotPoseSupplier = robotPoseSupplier;
        this.elevatorHeightSupplier = elevatorHeightSupplier;
    }
    
    @Override
    public void setupTracking() {
        new Trigger(hasPieceSupplier)
            .whileTrue(Commands.runOnce(this::updatePiecePosition)
                .ignoringDisable(true).repeatedly())
            .onFalse(Commands.runOnce(this::handlePlacement)
                .ignoringDisable(true));
    }
    
    /**
     * Update the position of the held coral piece based on robot/arm position
     */
    private void updatePiecePosition() {
        RobotContainer.placedCoral[0] = new Pose3d(robotPoseSupplier.get())
            .plus(new Transform3d(new Pose3d(), RobotContainer.components3d[LoggingConstants.WRIST_INDEX]))
            .plus(new Transform3d(new Pose3d(), new Pose3d(LoggingConstants.CORAL_OFFSET, new Rotation3d())));
    }
    
    /**
     * Handle the placement of a coral piece when it's released
     */
    private void handlePlacement() {
        // Calculate end effector position
        Pose3d endEffectorPose = getCurrentEndEffectorPose();
        
        // Find the closest scoring node
        Pose3d scoringNode = PoseCalculations.getClosestCoralScoringNode(endEffectorPose);
        
        // Manage placement indices with wraparound
        if (RobotContainer.placedCoralIndex >= RobotContainer.placedCoral.length) {
            RobotContainer.placedCoralIndex = 1; // Reserve index 0 for currently held piece
        }
        
        // Create start pose with alliance orientation
        Pose3d startPose = createStartPose();
        
        // Animate the piece placement
        animatePiecePlacement(startPose, scoringNode);
        
        // Increment index and hide held piece
        RobotContainer.placedCoralIndex++;
        hidePiece();
    }
    
    /**
     * Get the current position of the end effector
     */
    private Pose3d getCurrentEndEffectorPose() {
        double elevatorHeight = elevatorHeightSupplier.getAsDouble();
        Pose2d robotPose = robotPoseSupplier.get();
        return new Pose3d(
            robotPose.getTranslation().getX(),
            robotPose.getTranslation().getY(), 
            elevatorHeight, 
            new Rotation3d()
        );
    }
    
    /**
     * Create the starting pose for the piece animation
     */
    private Pose3d createStartPose() {
        return new Pose3d(
            RobotContainer.placedCoral[0].getTranslation(),
            new Rotation3d(
                RobotContainer.placedCoral[0].getRotation().getX(),
                RobotContainer.placedCoral[0].getRotation().getY() + 
                    Units.degreesToRadians(Robot.isRedAlliance() ? 180 : 0),
                RobotContainer.placedCoral[0].getRotation().getZ()
            )
        );
    }
    
    /**
     * Animate the piece moving from start to end position
     */
    private void animatePiecePlacement(Pose3d startPose, Pose3d endPose) {
        PieceThrower.throwPiece(
            () -> startPose,
            () -> endPose,
            (pose) -> RobotContainer.placedCoral[RobotContainer.placedCoralIndex] = pose,
            PIECE_ANIMATION_DURATION
        ).schedule();
    }
    
    /**
     * Hide the currently held piece by moving it below the field
     */
    private void hidePiece() {
        RobotContainer.placedCoral[0] = new Pose3d(
            0, 0, 
            -FieldConstants.CORAL_RADIUS_METERS + HIDDEN_Z_OFFSET, 
            new Rotation3d()
        );
    }
}
