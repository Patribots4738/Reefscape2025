package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {

    @AutoLog
    class VisionIOInputs {

        public Pose2d frontRobotPose = new Pose2d();
        public boolean frontRobotPoseValid = false;
        public double frontTimestampSeconds = 0.0;
        public double frontAverageTA = 0.0;
        public double frontAverageTD = 0.0;
        public int frontTagCount = 0;
        public int[] frontIds = new int[0];

        public Pose2d backRobotPose = new Pose2d();
        public boolean backRobotPoseValid = false;
        public double backTimestampSeconds = 0.0;
        public double backAverageTA = 0.0;
        public double backAverageTD = 0.0;
        public int backTagCount = 0;
        public int[] backIds = new int[0];

    }

    public default void updateInputs(VisionIOInputs inputs) {}
    
    public default void setFrontPipelineIndex(double index) {}

    public default void setBackPipelineIndex(double index) {}

    public default void setMegaTag2(boolean megaTag2) {}
    
}
