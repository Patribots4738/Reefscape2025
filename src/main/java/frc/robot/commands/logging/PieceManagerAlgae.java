package frc.robot.commands.logging;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.calc.PoseCalculations;

/**
 * Implementation of PieceManagerIO for algae pieces
 */
public class PieceManagerAlgae implements PieceManagerIO {
    private final BooleanSupplier hasPieceSupplier;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final DoubleSupplier elevatorHeightSupplier;
    
    // Constants 
    private static final double HIDDEN_Z_OFFSET = -0.05;
    private static final Pose3d ALGAE_DISPOSAL_POSE = new Pose3d(8.775, 1.9, 2.1175, new Rotation3d());
    private static final double HIDDEN_X_OFFSET = 0.5;
    
    /**
     * Create a new algae piece manager
     */
    public PieceManagerAlgae(BooleanSupplier hasPieceSupplier, 
                           Supplier<Pose2d> robotPoseSupplier,
                           DoubleSupplier elevatorHeightSupplier) {
        this.hasPieceSupplier = hasPieceSupplier;
        this.robotPoseSupplier = robotPoseSupplier;
        this.elevatorHeightSupplier = elevatorHeightSupplier;
    }
    
    @Override
    public void setupTracking() {
        new Trigger(hasPieceSupplier)
            .onTrue(Commands.runOnce(this::handleRemoval))
            .whileTrue(createTrackingAnimation())
            .onFalse(createDisposalAnimation());
    }
    
    /**
     * Create a command to continuously track the algae position while held
     */
    private Command createTrackingAnimation() {
        return PieceThrower.throwPiece(
                () -> getClosestRemovalNode(),
                this::getPiecePosition,
                (pose) -> RobotContainer.placedAlgae[0] = pose
            )
            .andThen(Commands.runOnce(() -> RobotContainer.placedAlgae[0] = getPiecePosition())
                .ignoringDisable(true).repeatedly());
    }
    
    /**
     * Create a command to animate the disposal of the algae
     */
    private Command createDisposalAnimation() {
        return PieceThrower.throwPiece(
                () -> RobotContainer.placedAlgae[0],
                () -> ALGAE_DISPOSAL_POSE,
                (pose) -> RobotContainer.placedAlgae[0] = pose
            )
            .andThen(Commands.runOnce(this::hidePiece)
                .ignoringDisable(true));
    }
    
    /**
     * Handle removal of algae from its position on the field
     */
    private void handleRemoval() {
        Pose3d endEffectorPose = getCurrentEndEffectorPose();
        Pose3d removalNode = PoseCalculations.getClosestAlgaeRemovalNode(endEffectorPose);
        
        // Find which algae node was removed and hide it
        for (int i = 1; i < RobotContainer.placedAlgae.length; i++) {
            if (i - 1 < FieldConstants.ALGAE_REMOVAL_LOCATIONS_ARRAY.length &&
                removalNode.equals(FieldConstants.ALGAE_REMOVAL_LOCATIONS_ARRAY[i - 1])) {
                hideAlgaeAtIndex(i);
                break;
            }
        }
    }
    
    /**
     * Get the current end effector position
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
     * Get the current position of the held algae piece
     */
    private Pose3d getPiecePosition() {
        return new Pose3d(robotPoseSupplier.get())
            .plus(new Transform3d(new Pose3d(), RobotContainer.components3d[LoggingConstants.WRIST_INDEX]))
            .plus(new Transform3d(new Pose3d(), new Pose3d(LoggingConstants.ALGAE_OFFSET, new Rotation3d())));
    }
    
    /**
     * Get the closest algae removal node based on current position
     */
    private Pose3d getClosestRemovalNode() {
        return PoseCalculations.getClosestAlgaeRemovalNode(
            getCurrentEndEffectorPose(), 
            false
        );
    }
    
    /**
     * Hide a specific algae piece by index
     */
    private void hideAlgaeAtIndex(int index) {
        RobotContainer.placedAlgae[index] = new Pose3d(
            HIDDEN_X_OFFSET, 0, 
            -FieldConstants.ALGAE_RADIUS_METERS + HIDDEN_Z_OFFSET, 
            new Rotation3d()
        );
    }
    
    /**
     * Hide the currently held piece by moving it below the field
     */
    private void hidePiece() {
        RobotContainer.placedAlgae[0] = new Pose3d(
            HIDDEN_X_OFFSET, 0, 
            -FieldConstants.ALGAE_RADIUS_METERS + HIDDEN_Z_OFFSET, 
            new Rotation3d()
        );
    }
}
