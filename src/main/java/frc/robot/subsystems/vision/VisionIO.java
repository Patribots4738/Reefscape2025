package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {

    @AutoLog
    class VisionIOInputs {

        public Pose2d robotPose = Pose2d.kZero;
        public boolean robotPoseValid = false;
        public double timestampSeconds = 0.0;
        public double averageTA = 0.0;
        public double averageTD = 0.0;
        public int[] tagIds = new int[0];

    }

    public default void updateInputs(VisionIOInputs inputs) {}
    
    public default void setPipelineIndex(int index) {}

    public default void setThrottle(int throttle) {}

    public default void setRobotOrientation(double yawDegrees) {}

    public default void setUseMegaTag2(boolean megaTag2) {}

    public default void setUsedTags(int[] tagIds) {}
    
}