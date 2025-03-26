package frc.robot.commands.logging;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Main factory class for creating piece managers
 */
public class PieceManager {
    
    /**
     * Create a coral piece manager
     * 
     * @param hasPieceSupplier Function that returns true when robot has a coral piece
     * @param robotPoseSupplier Function that provides the current robot pose
     * @param elevatorHeightSupplier Function that provides the current elevator height
     * @return A configured coral piece manager
     */
    public static PieceManagerIO forCoral(BooleanSupplier hasPieceSupplier, 
                                        Supplier<Pose2d> robotPoseSupplier,
                                        DoubleSupplier elevatorHeightSupplier) {
        return new PieceManagerCoral(hasPieceSupplier, robotPoseSupplier, elevatorHeightSupplier);
    }
    
    /**
     * Create an algae piece manager
     * 
     * @param hasPieceSupplier Function that returns true when robot has an algae piece
     * @param robotPoseSupplier Function that provides the current robot pose
     * @param elevatorHeightSupplier Function that provides the current elevator height
     * @return A configured algae piece manager
     */
    public static PieceManagerIO forAlgae(BooleanSupplier hasPieceSupplier, 
                                        Supplier<Pose2d> robotPoseSupplier,
                                        DoubleSupplier elevatorHeightSupplier) {
        return new PieceManagerAlgae(hasPieceSupplier, robotPoseSupplier, elevatorHeightSupplier);
    }
}
